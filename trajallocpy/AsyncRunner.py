"""Decentralized asynchronous execution path for ACBBA.

There is **no global barrier and no shared message pool**: every agent runs an
independent event loop (check inbox -> run consensus action rules -> rebuild its
bundle if needed -> broadcast changed bids to graph neighbours through the
simulated :mod:`trajallocpy.Transport`). Termination is decided by a
distributed quiescence detector (Dijkstra-Scholten / double-counting style),
not by counting consensus rounds.

Two execution modes share the exact same per-agent step:

* ``mode="step"`` (default) -- a single-thread discrete-event simulation over a
  :class:`Transport.LogicalClock`. Fully deterministic given a seed (delivery
  order is the transport's ``(release_time, seq)`` heap order and agents are
  activated in id order), so it is what the tests assert on.
* ``mode="threads"`` -- one real :class:`threading.Thread` per agent plus a
  coordinator thread modelling the network medium and the safety timeout, for
  realistic concurrency. Threads (not processes) because, once obstacle-aware
  pathfinding is live, each agent holds an ``extremitypathfinder`` environment
  that is not reliably picklable, and threads make delivery order seedable.
"""

import threading
import time

from trajallocpy import Transport
from trajallocpy._logging import logger


def _snapshot(agent):
    return (
        dict(agent.winning_bids),
        dict(agent.winning_agents),
        dict(agent.t),
        list(agent.bundle),
    )


class _Worker:
    """Per-agent event-loop step, shared by both execution modes."""

    def __init__(self, agent, mailbox, transport, refresh_period, stable_K):
        self.agent = agent
        self.mailbox = mailbox
        self.transport = transport
        self.refresh_period = refresh_period
        self.stable_K = stable_K
        self.idle_rounds = 0
        self.activations = 0

    def step(self, now):
        """One activation. Returns True if the agent did anything observable."""
        agent = self.agent
        before = _snapshot(agent)

        rebroadcasts = []
        for batch in self.mailbox.get_nowait_all():
            rebroadcasts.extend(agent.update_task_async(batch))

        # Replan: the first activation seeds a bundle; later ones rebuild after
        # consensus may have released part of the bundle.
        agent.build_bundle()

        changed = _snapshot(agent) != before
        did_refresh = self.refresh_period and self.activations % self.refresh_period == 0

        to_send = []
        if changed or did_refresh:
            to_send = agent.send_message()
        elif rebroadcasts:
            to_send = rebroadcasts

        if to_send:
            self.transport.broadcast(agent.id, to_send, t=now)

        self.activations += 1
        active = bool(changed or rebroadcasts or to_send)
        self.idle_rounds = 0 if active else self.idle_rounds + 1
        return active

    @property
    def idle(self):
        return self.idle_rounds >= self.stable_K


def _quiescent(workers, transport):
    if transport.in_flight() != 0:
        return False
    if transport.sent != transport.received:
        return False
    return all(w.idle for w in workers)


def run(
    runner,
    mode="step",
    max_runtime=30.0,
    tick=0.01,
    refresh_period=8,
    stable_K=3,
    sleep=0.001,
):
    """Drive ``runner.robot_list`` (ACBBA agents) to a conflict-free allocation.

    Populates the same attributes the synchronous ``solve`` would
    (``start_time``/``end_time``/``iterations``/``converged``) so
    ``evaluateSolution`` works unchanged.
    """
    agents = list(runner.robot_list.values())
    transport = runner.transport_layer
    workers = []
    for agent in agents:
        mailbox = transport.register(agent.id)
        workers.append(_Worker(agent, mailbox, transport, refresh_period, stable_K))

    runner.start_time = timeit_now()
    if mode == "step":
        iterations = _run_step(workers, transport, max_runtime, tick)
    elif mode == "threads":
        iterations = _run_threads(workers, transport, max_runtime, sleep)
    else:
        raise ValueError(f"unknown async mode: {mode!r}")
    runner.end_time = timeit_now()

    runner.iterations = iterations
    runner.converged = _quiescent(workers, transport)
    if not runner.converged:
        logger.warning("ACBBA stopped on the %.1fs safety timeout without quiescence", max_runtime)
    return runner


def timeit_now():
    import timeit

    return timeit.default_timer()


def _run_step(workers, transport, max_runtime, tick):
    clock = transport.clock
    if not isinstance(clock, Transport.LogicalClock):
        clock = Transport.LogicalClock()
        transport.clock = clock
    rounds = 0
    max_rounds = max(1, int(max_runtime / tick))
    while rounds < max_rounds:
        now = clock()
        transport.deliver_due(now)
        for worker in workers:  # deterministic: id order
            worker.step(now)
        rounds += 1
        if _quiescent(workers, transport):
            break
        clock.advance(tick)
    return rounds


def _run_threads(workers, transport, max_runtime, sleep):
    stop = threading.Event()
    rounds = {"n": 0}

    def agent_loop(worker):
        while not stop.is_set():
            worker.step(transport.clock())
            time.sleep(sleep)

    threads = [threading.Thread(target=agent_loop, args=(w,), daemon=True) for w in workers]
    for thread in threads:
        thread.start()

    deadline = time.monotonic() + max_runtime
    while time.monotonic() < deadline:
        transport.deliver_due()
        rounds["n"] += 1
        if _quiescent(workers, transport):
            break
        time.sleep(sleep)
    stop.set()
    for thread in threads:
        thread.join(timeout=1.0)
    transport.deliver_due()
    return rounds["n"]

"""Simulated asynchronous message transport (stdlib only).

This module models the *network medium* between decentralized agents so that
ACBBA can run without a global barrier or a shared message pool. It provides:

* :class:`CommunicationGraph` -- static, directed or undirected, and optionally
  time-varying (dynamic) topology.
* :class:`LinkModel` -- per-link simulated latency and Bernoulli packet loss.
* :class:`Transport` -- in-memory delayed delivery via a lock-guarded heap with
  deterministic ``(release_time, seq)`` ordering and per-directed-link seeded
  RNGs, so a run is reproducible given a seed independent of thread scheduling.
* :class:`Mailbox` -- an agent's inbox.
* :class:`LogicalClock` -- a deterministic clock for the step-driven test mode.

Agent ids are assumed to be the graph indices ``0..n-1`` (matching the rest of
the library, which indexes ``communication_graph`` by ``agent.id``).
"""

import heapq
import queue
import random
import threading
import time
from dataclasses import dataclass

import numpy as np


@dataclass
class LinkModel:
    """Per-link network characteristics (all times in simulated seconds)."""

    latency_mean: float = 0.0
    latency_jitter: float = 0.0  # uniform +/- around the mean
    loss_prob: float = 0.0  # Bernoulli drop probability per message


class LogicalClock:
    """Monotone clock advanced explicitly; used by the deterministic step mode."""

    def __init__(self, start: float = 0.0):
        self._t = float(start)

    def __call__(self) -> float:
        return self._t

    def advance(self, dt: float) -> float:
        self._t += dt
        return self._t


class Mailbox:
    """Thread-safe inbox for a single agent."""

    def __init__(self):
        self._q = queue.Queue()

    def put(self, msg):
        self._q.put(msg)

    def get_nowait_all(self) -> list:
        out = []
        while True:
            try:
                out.append(self._q.get_nowait())
            except queue.Empty:
                break
        return out

    def empty(self) -> bool:
        return self._q.empty()


class CommunicationGraph:
    """Directed/undirected agent connectivity, optionally time-varying."""

    def __init__(self, n, adjacency=None, *, directed=False):
        self.n = n
        self.directed = directed
        self._schedule = None
        if adjacency is None:
            adjacency = np.ones((n, n)) - np.eye(n)
        adjacency = np.asarray(adjacency, dtype=float).copy()
        np.fill_diagonal(adjacency, 0)
        if not directed:
            adjacency = np.maximum(adjacency, adjacency.T)
        self._adj = adjacency

    @classmethod
    def full(cls, n):
        return cls(n)

    @classmethod
    def from_matrix(cls, matrix, directed=False):
        matrix = np.asarray(matrix, dtype=float)
        return cls(matrix.shape[0], matrix, directed=directed)

    @classmethod
    def ring(cls, n, directed=False):
        adjacency = np.zeros((n, n))
        for i in range(n):
            adjacency[i, (i + 1) % n] = 1
            adjacency[i, (i - 1) % n] = 1
        return cls(n, adjacency, directed=directed)

    @classmethod
    def line(cls, n):
        adjacency = np.zeros((n, n))
        for i in range(n - 1):
            adjacency[i, i + 1] = 1
            adjacency[i + 1, i] = 1
        return cls(n, adjacency)

    def with_schedule(self, schedule):
        """``schedule`` is ``callable(t) -> adjacency`` for dynamic topology."""
        self._schedule = schedule
        return self

    def _matrix(self, t):
        if self._schedule is None:
            return self._adj
        matrix = np.asarray(self._schedule(t), dtype=float).copy()
        np.fill_diagonal(matrix, 0)
        if not self.directed:
            matrix = np.maximum(matrix, matrix.T)
        return matrix

    def neighbors(self, agent_id, t=0.0) -> list:
        row = self._matrix(t)[agent_id]
        return [j for j in range(self.n) if j != agent_id and row[j] > 0]

    def is_connected(self, src, dst, t=0.0) -> bool:
        return bool(self._matrix(t)[src][dst] > 0)


class Transport:
    """In-memory network: queues messages with simulated latency/loss."""

    def __init__(self, graph: CommunicationGraph, link=None, seed=None, clock=None):
        self.graph = graph
        self.link = link if link is not None else LinkModel()
        self._base_seed = 0 if seed is None else int(seed)
        self.clock = clock if clock is not None else time.monotonic
        self._mailboxes = {}
        self._heap = []  # (release_time, seq, dst, msg)
        self._seq = 0
        self._lock = threading.Lock()
        self._link_rng = {}
        # Counters used by the distributed termination detector.
        self.dropped = 0
        self.sent = 0
        self.received = 0

    def register(self, agent_id) -> Mailbox:
        mailbox = Mailbox()
        self._mailboxes[agent_id] = mailbox
        return mailbox

    def _rng(self, src, dst) -> random.Random:
        key = (src, dst)
        rng = self._link_rng.get(key)
        if rng is None:
            rng = random.Random((self._base_seed * 1000003) ^ (hash(key) & 0xFFFFFFFF))
            self._link_rng[key] = rng
        return rng

    def _link_for(self, src, dst) -> LinkModel:
        if isinstance(self.link, dict):
            return self.link.get((src, dst)) or self.link.get(None) or LinkModel()
        return self.link

    def _enqueue(self, src, dst, msg, now):
        link = self._link_for(src, dst)
        rng = self._rng(src, dst)
        if link.loss_prob > 0 and rng.random() < link.loss_prob:
            with self._lock:
                self.dropped += 1
            return
        latency = link.latency_mean
        if link.latency_jitter > 0:
            latency += rng.uniform(-link.latency_jitter, link.latency_jitter)
        release = now + max(0.0, latency)
        with self._lock:
            heapq.heappush(self._heap, (release, self._seq, dst, msg))
            self._seq += 1
            self.sent += 1

    def send(self, src, dst, msg, t=None):
        now = self.clock() if t is None else t
        if self.graph.is_connected(src, dst, now):
            self._enqueue(src, dst, msg, now)

    def broadcast(self, src, msg, t=None):
        now = self.clock() if t is None else t
        for dst in self.graph.neighbors(src, now):
            self._enqueue(src, dst, msg, now)

    def deliver_due(self, now=None) -> int:
        if now is None:
            now = self.clock()
        delivered = 0
        with self._lock:
            while self._heap and self._heap[0][0] <= now:
                _, _, dst, msg = heapq.heappop(self._heap)
                mailbox = self._mailboxes.get(dst)
                if mailbox is not None:
                    mailbox.put(msg)
                    self.received += 1
                    delivered += 1
        return delivered

    def in_flight(self) -> int:
        with self._lock:
            return len(self._heap)

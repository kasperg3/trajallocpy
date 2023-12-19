import pytest


def dummy_test():
    assert 1 == 1


# Run the tests
if __name__ == "__main__":
    pytest.main(["-v", "-x", "tests/allocation_test.py"])

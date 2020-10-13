import time
import threading

def create_thread(
    timer_period_sec: float,
    # callback: Callable,
    callback,
    # ) -> Thread:
    ):
    """
    Create a new Thread.
    """
    def thread_loop(timer_period_sec):
        while True:
            callback()
            time.sleep(timer_period_sec)

    thread = threading.Thread(
        target=thread_loop, daemon=True, args=(timer_period_sec,))
    return thread

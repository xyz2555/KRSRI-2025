import threading
import time

def loop1():
    for i in range(5):
        print("Thread 1")
        time.sleep(1)

def loop2():
    for i in range(5):
        print("Thread 2")
        time.sleep(1)
if __name__ =="__main__":
    t1 = threading.Thread(target=loop1)
    t2 = threading.Thread(target=loop2)

    t1.start()
    t2.start()

    t2.join()
    t1.join()
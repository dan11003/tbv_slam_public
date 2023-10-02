# This Python file uses the following encoding: utf-8

# if__name__ == "__main__":
#     pass
import time
from threading import Thread
from threading import Lock


mutex = threading.Lock()
list=CreateAllJobs()
last_free_job=0

def GetJob():
    mutex.acquire()
    job = list(last_free_job)
    print("next job is"+str(job))
    last_free_job = last_free_job+1
    mutex.release()
    return list(last_free_job)


def thread(thread_id)
    print("started thread with id"+str(thread_id))

    while(true)
    job=GetJob()
    if(job != empty):
        rosrun package program_name JobToArgument(job)
    else:
        break


    def main:
      Startkthreds(k)
      threads.join()
      exit()








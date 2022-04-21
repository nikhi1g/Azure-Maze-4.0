from EmailModule.Email import Email
from time import sleep
from random import random
print("Starting Up Kinetic Mail!")
pairsList = None
from threading import Thread




def sort_storage():
    global pairsList
    print("Logged into kineticmaze@gmail.com, check this after May 30, 2022. \nDelete anything to do with this code "
          "after that, if this breaks.")
    kmail = Email("kineticmaze@gmail.com", "kineticmaze7266!")
    while True:
        scores = []
        names = []
        with open("storage.txt", "r") as file:
            for line in file:
                split_line = line.strip().split()
                scores.append(split_line[0])
                names.append(split_line[1])

        pairs = list(zip(scores, names))
        pairs.sort(key=lambda pair: int(pair[0]))

        count = 1
        score_board = ""
        with open("storage.txt", "r") as f:
            leader_length = len(f.readlines())

        while count < leader_length:
            score_board += str(count) + ".  " + pairs[count][0] + " " + pairs[count][1] + "\n"
            count += 1

        pairsList = score_board
        try:
            kmail.checkForEmail("scores", f"Hello from the Kinetic Maze! {str(random())}",
                                "Here are your top scorers:\n" + str(pairsList))
        except Exception as e: pass
if __name__ == '__main__':
    sort_storage()

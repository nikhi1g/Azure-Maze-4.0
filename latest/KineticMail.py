from EmailModule.Email import Email

pairsList = None


def sort_storage():
    global pairsList
    scores = []
    names = []
    with open("storage.txt", "r") as file:
        for line in file:
            split_line = line.strip().split()
            scores.append(split_line[0])
            names.append(split_line[1])
    pairs = list(zip(scores, names))
    pairs.sort(key=lambda pair: int(pair[0]))
    pairsList = dict(pairs)


if __name__ == '__main__':
    sort_storage()
    kmail = Email("kineticmaze@gmail.com", "kineticmaze7266!")
    kmail.checkForEmailConstantly("scores", "Hello from the Kinetic Maze!",
                                  "Here are your top scorers:\n" + str(pairsList))

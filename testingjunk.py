from time import sleep
#
# with open("latest/storage.txt") as f:
#     last_line = f.readlines()[-1]
#     space_index = last_line.index(" ")

with open("latest/storage.txt", "r+") as f:
    if len(f.readlines()[-1]) > 0:
        f.write("\n")

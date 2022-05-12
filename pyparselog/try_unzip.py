from zipfile import ZipFile

with ZipFile("./log/no_move_to_a_prime/not go to A prime pattern 2 test2.zip", 'r') as zipObj:
    # Extract all the contents of zip file in current directory
    zipObj.extractall("./log/no_move_to_a_prime/")
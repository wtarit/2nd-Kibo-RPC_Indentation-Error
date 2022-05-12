from datetime import datetime
import glob
import os
import json
from zipfile import ZipFile

key_to_track = ["QR[raw data]:", "QR[count]", "QR[total_time]", "AR[pixelperM]", "AR[count]", "AR[total_time]", "AR[target_pos]", "AR[target_point(xy plane)]", "AR[Robot Position]"]

if not os.path.isfile("result.tsv"):
    output = open("result.tsv", "a")
    output.write("log name\t")
    header = key_to_track + ["x_error", "y_error", "total_error", "PointA_time", "Snapshot_time", "Mission_time"]
    for k in header:
        output.write(k)
        output.write("\t")
    output.write("\n")

basepath = "./log"
zip_folder = glob.glob(f"{basepath}/*.zip")

output = open("result.tsv", "a")

for zipped_file in zip_folder:
    dir_name = os.path.splitext(zipped_file)[0]
    if os.path.isdir(dir_name):
        continue
    else:
        os.mkdir(dir_name)
        #get the folder name
        toappend = f"\"{os.path.basename(dir_name)}\""
        with ZipFile(zipped_file, 'r') as zipObj:
            # Extract all the contents of zip file in current directory
            zipObj.extractall(dir_name)
        #parse adb log file
        with open(f"{dir_name}/adb.log", "r", encoding="utf8", errors='ignore') as adb:
            for line in adb:
                stripped_line = line.strip()
                if any(substring in stripped_line for substring in key_to_track):
                    # print(list(substring in stripped_line for substring in key_to_track))
                    # s = stripped_line.rsplit(":", 1)[-1]
                    s = stripped_line.split(":", 4)[-1]
                    # print(s)
                    toappend += f"\t\"{s}\""
        
        with open(f"{dir_name}/result.json", "r") as result:
            d = json.load(result)
            Xlist = []
            Ylist = []
            Rlist = []
            approach = d["Approach"][0]
            for i in approach:
                Xlist.append(approach[i]["x"] * 100)
                Ylist.append(approach[i]["y"] * 100)
                Rlist.append(approach[i]["r"] * 100)
                
            fmt = '%Y%m%d %H%M%S%f'
            startT = datetime.strptime(d["Mission Time"]["start"], fmt)
            stopT = datetime.strptime(d["Mission Time"]["finish"], fmt)
            qrT = datetime.strptime(d["QR"]["0"]["timestamp"], fmt)
            targetT = datetime.strptime(d["Approach"][0]["9"]["timestamp"], fmt)

        toappend += f"\t{sum(Xlist)/len(Xlist)}\t{sum(Ylist)/len(Ylist)}\t{sum(Rlist)/len(Rlist)}"
        qrTdiff = str(qrT - startT).split(":",1)[1]
        targetTdiff = str(targetT - startT).split(":",1)[1]
        missionTdiff = str(stopT - startT).split(":",1)[1]
        toappend += f"\t{qrTdiff}\t{targetTdiff}\t{missionTdiff}"
        print(toappend)
        output.write(f"{toappend}\n")

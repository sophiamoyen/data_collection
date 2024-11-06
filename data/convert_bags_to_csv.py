import bagpy
from bagpy import bagreader
from os import listdir, getcwd
from os.path import isfile, join


def get_bag_files_from_folder(mypath):
    onlyfiles = [f for f in listdir(mypath) if isfile(join(mypath, f))]
    bag_files = []
    for f in onlyfiles:
        if f.endswith('.bag'):
            bag_files.append(f)
    return bag_files

def read_bag_to_df_csv(bag_file):
    print(bag_file)
    bag = bagreader(bag_file)
    csvfiles = []
    for t in bag.topics:
        data = bag.message_by_topic(t)
        csvfiles.append(data)


# Code :
participant_folder = input("Please enter the folder name of the participant: ")
current_folder = getcwd()
bagfiles_list = get_bag_files_from_folder(current_folder+"/"+participant_folder)

for bagfile in bagfiles_list:
    read_bag_to_df_csv(current_folder+"/"+participant_folder+"/"+bagfile)
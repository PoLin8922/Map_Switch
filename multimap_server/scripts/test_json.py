import json
import numpy as np

def main():
    f = open('switcher_env.json')
    data = json.load(f)

    print(type(data))
    print(type(data['first_a']['initial_pose']))

    initial_pose = data['first_a']['initial_pose']
    print(initial_pose[0])

    # print(data)

    # for i in data['initial_pose']:
        # print(i)
    '''
    for i in data['initial_pose']:
        print(i)'''

    f.close()

if __name__ == "__main__":
    main()
    
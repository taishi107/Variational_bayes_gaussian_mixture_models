import csv

with open('sensor_data.csv','w') as f:
    t = [[0,0],[1,1],[2,2]]
    writer = csv.writer(f, lineterminator='\n')
    writer.writerows(t)


import csv

filename = 'sensor_values.bag'
bag = rosbag.Bag(filename)
print('-------------------------------')
print('Start converting %s to a CSV file')

right_forward = []
right_side = []
left_side = []
left_forward = []
sensor_values = []
    
for topic, msg, t in bag.read_messages(topics=['/event']):
    right_forward.append(float(msg.right_forward))
    right_side.append(float(msg.right_side))
    left_side.append(float(msg.left_side))
    left_forward.append(float(msg.left_forward))
    sensor_values.append(left_forward,left_side,right_side,right_forward)

with open('sensor_data.csv','w') as f:    
    writer = csv.writer(f, lineterminator='\n')\
    writer.writerows(sensor_values)


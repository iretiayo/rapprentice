import rosbag
import sys

if len(sys.argv) < 3:
	print "Usage: python merge_bags.py [bag1 to merge] [bag 2 to merge] [output bag]"
	raise Exception("Invalid number of params")
bag1_name = sys.argv[1]
bag2_name = sys.argv[2]
output_bag_name = sys.argv[3]
print "Merging ", bag1_name, "and ", bag2_name, "into ", output_bag_name

output_bag = rosbag.Bag(output_bag_name, 'w')
bag1 = rosbag.Bag(bag1_name)
bag2 = rosbag.Bag(bag2_name)

for topic, msg, t in bag1.read_messages():
	output_bag.write(topic, msg, t=t)
output_bag.flush()

for topic, msg, t in bag2.read_messages():
	output_bag.write(topic, msg, t=t)
output_bag.flush()

bag1.close()
bag2.close()
output_bag.close()


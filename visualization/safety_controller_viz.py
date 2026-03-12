# import rosbag2_py
# import matplotlib.pyplot as plt

# def extract(bagfile, topic):
#     bag = rosbag.Bag(bagfile)
#     t = []
#     v = []

#     for _, msg, ts in bag.read_messages(topics=[topic]):
#         t.append(ts.to_sec())
#         v.append(msg.data)

#     bag.close()

#     t0 = t[0]
#     t = [x - t0 for x in t]
#     return t, v


# bags = [
#     "wf_speedtest1_real_trial1.bag",
#     "wf_speedtest1_real_trial2.bag",
#     "wf_speedtest1_real_trial3.bag"
# ]

# for b in bags:
#     t, v = extract(b, "/speed")
#     plt.plot(t, v, label=b)

# plt.xlabel("Time (s)")
# plt.ylabel("Speed")
# plt.legend()
# plt.show()
import rosbag2_py
import matplotlib.pyplot as plt
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def extract(bag_path, topic, ignore_first_sec=0, max_time=None):

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=bag_path, storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("cdr", "cdr"),
    )

    topic_types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    msg_type = get_message(topic_types[topic])

    t, v = [], []

    while reader.has_next():
        topic_name, data, ts = reader.read_next()

        if topic_name != topic:
            continue

        msg = deserialize_message(data, msg_type)

        t.append(ts / 1e9)
        v.append(msg.data)

    if not t:
        return [], []

    t0 = t[0]
    print(f'{t0=}')
    t = [x - t0 for x in t]

    # ignore first x seconds
    filtered_t = []
    filtered_v = []

    for ti, vi in zip(t, v):
        if ti < ignore_first_sec:
            continue
        if max_time is not None and ti -ignore_first_sec > max_time:
            break   # safe since time is increasing
        filtered_t.append(ti - ignore_first_sec)
        filtered_v.append(vi-1.8)

    return filtered_t, filtered_v


bags = [
    # "ros_bags/only_wf/onlywf_45angle_trial1",
    # "ros_bags/only_wf/onlywf_45angle_trial2",
    # "ros_bags/only_wf/onlywf_45angle_trial3",

    "ros_bags/only_wf/onlywf_trial1",
    "ros_bags/only_wf/onlywf_trial2",
    "ros_bags/only_wf/onlywf_trial3",
    # "ros_bags/wf_speedtest/wf_speedtest3_real_trial1"
    # "ros_bags/wf_carpet/estvmeas_trial1",
    # "ros_bags/wf_carpet/estvmeas_trial2",
    # "ros_bags/wf_carpet/estvmeas_trial3",

]

ignore_first_seconds_ = [1, 5, 0, 0, ]

all_v = []

# for b, i in zip(bags, ignore_first_seconds_):
#     t, v = extract(b, "/observed_error", ignore_first_sec = i, max_time=13)
#     all_v.append(v)


#     legend_label = "Trial " + b[-1]
#     if "45" in b:
#         plt.plot(t, v, label=" $\\frac{\pi}{4}$ rad " + legend_label, color='green')
#     else :
#         plt.plot(t, v, label="0 rad " + legend_label, color='blue', linestyle="-")

# for b, k in zip(bags, ignore_first_seconds_):
#     t, v = extract(b, "/vesc/low_level/input/safety", ignore_first_sec=k)
#     all_v.append(v)

# # v_straight = [(a + b +c)/3 for a,b,c in zip(*all_v[:3])]
# # v_angled = [(x+y+z)/3 for x,y,z in zip(*all_v[3:])]
# # plt.plot(t, v_straight, label="$\\frac{\pi}{4}$ rad", color='green')
# # plt.plot(t, v_angled, label="0 rad", color='blue')

#     steering_rate = []
#     rate_time = []

#     for i in range(1, len(v)):
#         dv = v[i] - v[i-1]
#         dt = t[i] - t[i-1]

#         steering_rate.append(dv / dt)
#         rate_time.append(t[i])

#     plt.plot(rate_time, steering_rate, label=b)
for b, ignore in zip(bags, ignore_first_seconds_):

    t, v = extract(b, "/observed_error", ignore_first_sec=ignore)

    # steering_rate = []
    # rate_time = []

    # for k in range(1, len(v)):
    #     dv = v[k] - v[k-1]
    #     dt = t[k] - t[k-1]

    #     steering_rate.append(dv / dt)
    #     rate_time.append(t[k])

    plt.plot(t, v, label="Trial " + b[-1])

plt.xlabel("Time (s)")
plt.ylabel("Desired Dist - LiDAR Dist to Wall (m)")
plt.title("Wall Follower Error on Concrete at 1m/s")
plt.legend()
plt.savefig("Wall Follower oscillations on concrete at v1.png")
plt.show()

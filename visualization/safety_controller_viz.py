import rosbag
import matplotlib.pyplot as plt

def extract(bagfile, topic):
    bag = rosbag.Bag(bagfile)
    t = []
    v = []

    for _, msg, ts in bag.read_messages(topics=[topic]):
        t.append(ts.to_sec())
        v.append(msg.data)

    bag.close()

    t0 = t[0]
    t = [x - t0 for x in t]
    return t, v


bags = [
    "wf_speedtest1_real_trial1.bag",
    "wf_speedtest1_real_trial2.bag",
    "wf_speedtest1_real_trial3.bag"
]

for b in bags:
    t, v = extract(b, "/speed")
    plt.plot(t, v, label=b)

plt.xlabel("Time (s)")
plt.ylabel("Speed")
plt.legend()
plt.show()


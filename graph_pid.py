import matplotlib.pyplot as plt
import seaborn as sns

left_values = []
right_values = []
with open("ticks_over_time.txt", "r") as f:
    for line in f:
        l, r = line.split(",")
        left_values.append(l)
        right_values.append(r)


g = sns.lineplot(x=range(len(left_values)), y=left_values)
plt.xlabel('X-axis Label')
plt.ylabel('Y-axis Label')
plt.title('Line Plot of Provided Data')
plt.ylim(0, 100)  # Set y-axis limits
plt.yticks(range(0, 91, 10))  # Set y-axis ticks
plt.show()

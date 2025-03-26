# import csv
# from datetime import datetime
# import matplotlib.pyplot as plt
# from collections import defaultdict


# def load_data_from_csv(filename):
#     """Load data from the CSV file."""
#     data = defaultdict(list)
#     try:
#         with open(filename, mode='r') as csvfile:
#             reader = csv.DictReader(csvfile)
#             for row in reader:
#                 # Parse the timestamp and group by seconds
#                 timestamp = datetime.strptime(row['timestamp'], '%Y-%m-%d %H:%M:%S.%f')
#                 second = timestamp.replace(microsecond=0)  # Group by second
#                 topic = row['topic']
#                 data[second].append(topic)
#     except FileNotFoundError:
#         print(f"Error: File '{filename}' not found.")
#         return None
#     return data


# def process_data(data):
#     """Process the data to count messages for each category."""
#     task_definition_counts = []
#     tasks_align_counts = []
#     other_ids_counts = []
#     time_points = []

#     for second, topics in sorted(data.items()):
#         time_points.append(second)
#         task_definition_counts.append(topics.count('mrs_tasks/task_definition'))
#         tasks_align_counts.append(topics.count('/mrs_main/tasks_align'))
#         other_ids_counts.append(sum(1 for topic in topics if topic.startswith('/mrs_main/id_')))

#     return time_points, task_definition_counts, tasks_align_counts, other_ids_counts


# def plot_data(time_points, task_definition_counts, tasks_align_counts, other_ids_counts):
#     """Plot the data."""
#     plt.figure(figsize=(10, 6))

#     plt.plot(time_points, task_definition_counts, label='mrs_tasks/task_definition', marker='o')
#     plt.plot(time_points, tasks_align_counts, label='/mrs_main/tasks_align', marker='o')
#     plt.plot(time_points, other_ids_counts, label='/mrs_main/id_*', marker='o')

#     plt.xlabel('Time (seconds)')
#     plt.ylabel('Message Count')
#     plt.title('Message Timeline')
#     plt.legend()
#     plt.grid(True)
#     plt.tight_layout()

#     plt.show()


# def main():
#     filename = 'msg_timeline.csv'
#     data = load_data_from_csv(filename)
#     if data is None:
#         return

#     time_points, task_definition_counts, tasks_align_counts, other_ids_counts = process_data(data)
#     plot_data(time_points, task_definition_counts, tasks_align_counts, other_ids_counts)


# if __name__ == '__main__':
#     main()

import csv
from datetime import datetime
from typing import Text
import matplotlib.pyplot as plt
from collections import defaultdict

from numpy import array


def load_data_from_csv(filename):
    """Load data from the CSV file."""
    data = defaultdict(list)
    try:
        with open(filename, mode='r') as csvfile:
            reader = csv.DictReader(csvfile)
            for row in reader:
                # Parse the timestamp and group by seconds
                timestamp = datetime.strptime(row['timestamp'], '%Y-%m-%d %H:%M:%S.%f')
                second = timestamp.replace(microsecond=0)  # Group by second
                topic = row['topic']
                data[second].append(topic)
    except FileNotFoundError:
        print(f"Error: File '{filename}' not found.")
        return None
    return data


def process_data(data):
    """Process the data to count messages for each category and calculate elapsed time in minutes."""
    task_definition_counts = []
    tasks_align_counts = []
    other_ids_counts = []
    elapsed_time = []

    sorted_data = sorted(data.items())
    if not sorted_data:
        return [], [], [], []

    start_time = sorted_data[0][0]  # First timestamp

    for second, topics in sorted_data:
        elapsed_time.append((second - start_time).total_seconds() / 60)  # Time elapsed in minutes
        task_definition_counts.append(topics.count('mrs_tasks/task_definition'))
        tasks_align_counts.append(topics.count('/mrs_main/tasks_align'))
        other_ids_counts.append(sum(1 for topic in topics if topic.startswith('/mrs_main/id_')))

    return elapsed_time, task_definition_counts, tasks_align_counts, other_ids_counts


def plot_data(elapsed_time, task_definition_counts, tasks_align_counts, other_ids_counts):
    """Plot the data."""
    fig= plt.figure(figsize=(10, 6))

    plt.plot(elapsed_time, task_definition_counts, label='mrs_tasks/task_definition', marker='o')
    plt.plot(elapsed_time, tasks_align_counts, label='/mrs_main/tasks_align', marker='o')
    plt.plot(elapsed_time, other_ids_counts, label='/mrs_main/id_*', marker='o')

    plt.xlabel('Time Elapsed (minutes)')
    plt.ylabel('Message Count')
    plt.title('Message Timeline (Time Elapsed in Minutes)')
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    # print(plt.xticks())
    # fig.(['-1:00', '0:00', '1:00', '2:00', '3:00', '4:00', '5:00', '6:00', '7:00', '8:00'])
    plt.show()


def main():
    filename = 'msg_timeline.csv'
    data = load_data_from_csv(filename)
    if data is None:
        return

    elapsed_time, task_definition_counts, tasks_align_counts, other_ids_counts = process_data(data)
    plot_data(elapsed_time, task_definition_counts, tasks_align_counts, other_ids_counts)


if __name__ == '__main__':
    main()
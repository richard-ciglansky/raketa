import csv
import matplotlib.pyplot as plt
from matplotlib.ticker import MultipleLocator

from kinematics import transform_to_si, normalize_time, normalize_data, transform_to_kinematics, average_data


def read_measurements(filename, breaking_id, breaking_time):
    measure = []

    with open(filename, newline='') as csvfile:
        spamreader = csv.reader(csvfile, delimiter=',', quotechar='|')
        next(spamreader) # Skip header
        for row in spamreader:
            measure.append([int(row[1]), int(row[2]), int(row[3]), int(row[4]), int(row[5]), int(row[6]), int(row[7])]);
    return measure


def data_pipeline(measurements, accRange, rotRange, offset):
    return transform_to_kinematics(
        normalize_time(
            #average_data(
            measurements
            #, 10)
        ),
        accRange,
        rotRange,
        offset)

measurements1 = read_measurements('motor_D9-7_start_1.csv', "190400", "0")
measurements2 = read_measurements('motor_D9-7_start_2.csv', "624995", "0")
measurements3 = read_measurements('raketa3.csv', "624995", "0")

offset1 = normalize_data(measurements1, 3000)
offset2 = normalize_data(measurements2, 3000)
offset3 = normalize_data(measurements3, 3000)

# Max value for 16-bit signed integer is 2^16 - 1 = 32767
MAX_16BIT = 32767.0

# Conversion factors
accel_factor = (16 * 9.81) / MAX_16BIT
rot_factor = 1000 / MAX_16BIT

measure1 = data_pipeline(measurements1, accel_factor, rot_factor, offset1)
measure2 = data_pipeline(measurements2, accel_factor, rot_factor, offset2)
measure3 = data_pipeline(measurements3, accel_factor, rot_factor, offset3)

def truncate_to_landing(data):
    if not data:
        return data
    max_idx = max(range(len(data)), key=lambda i: data[i][9])
    for j in range(max_idx + 1, len(data)):
        if data[j][9] <= 0:
            return data[:j + 1]
    return data

def truncate_to_start(data, accel_threshold=10.0):
    if not data:
        return data
    start_idx = 0
    for i, m in enumerate(data):
        if abs(m[3]) > accel_threshold:
            start_idx = i
            break
    trimmed = data[start_idx:]
    if not trimmed:
        return trimmed
    t0 = trimmed[0][0]
    return [[m[0] - t0] + list(m[1:]) for m in trimmed]

def prepare(data):
    return truncate_to_landing(truncate_to_start(data))

datasets = [
    {"label": "D9-7_start_1", "data": prepare(measure1)},
    {"label": "D9-7_start_2", "data": prepare(measure2)},
    {"label": "C6-7", "data": prepare(measure3)}
]

plt.figure(figsize=(20, 15))

def configure_time_axis(ax):
    ax.xaxis.set_major_locator(MultipleLocator(1))
    ax.xaxis.set_minor_locator(MultipleLocator(0.1))
    ax.grid(True, which='major', linewidth=0.8)
    ax.grid(True, which='minor', linewidth=0.4, linestyle=':')

# Subplot 1: Acceleration
ax1 = plt.subplot(3, 1, 1)
for ds in datasets:
    times = [m[0] / 1000.0 for m in ds["data"]]
    values = [m[3] for m in ds["data"]]
    ax1.plot(times, values, label=ds["label"])
ax1.set_ylabel('Acceleraration (m/s^2)')
ax1.set_title('Rocket Data: Acceleration, Velocity and Height')
configure_time_axis(ax1)
ax1.legend()


def annotate_maxes(ax, series):
    # series: list of (times, values, color)
    peaks = []
    for times, values, color in series:
        if not values:
            continue
        max_idx = max(range(len(values)), key=lambda i: values[i])
        peaks.append((times[max_idx], values[max_idx], color))

    peaks.sort(key=lambda p: p[1], reverse=True)

    fig = ax.get_figure()
    fig.canvas.draw()
    renderer = fig.canvas.get_renderer()

    ax_bbox = ax.get_window_extent(renderer=renderer)

    def candidate_offsets():
        step = 16
        for i in range(20):
            yield (8, 10 + i * step)      # up-right stack
        for i in range(20):
            yield (-80, 10 + i * step)    # up-left stack
        for i in range(20):
            yield (8, -18 - i * step)     # down-right stack
        for i in range(20):
            yield (-80, -18 - i * step)   # down-left stack

    placed_bboxes = []
    for tmax, vmax, color in peaks:
        ax.plot(tmax, vmax, 'o', color=color, markersize=4)
        text = f'{vmax:.2f} @ {tmax:.2f}s'
        best_ann = None
        best_bbox = None
        for dx, dy in candidate_offsets():
            ann = ax.annotate(text,
                              xy=(tmax, vmax),
                              xytext=(dx, dy), textcoords='offset points',
                              fontsize=11, color=color,
                              arrowprops=dict(arrowstyle='-', color=color, lw=0.5))
            fig.canvas.draw()
            bbox = ann.get_window_extent(renderer=renderer)
            inside = (bbox.x0 >= ax_bbox.x0 and bbox.x1 <= ax_bbox.x1
                      and bbox.y0 >= ax_bbox.y0 and bbox.y1 <= ax_bbox.y1)
            overlap = any(bbox.overlaps(b) for b in placed_bboxes)
            if inside and not overlap:
                best_ann = ann
                best_bbox = bbox
                break
            ann.remove()
        if best_ann is None:
            best_ann = ax.annotate(text,
                                   xy=(tmax, vmax),
                                   xytext=(8, 10), textcoords='offset points',
                                   fontsize=11, color=color,
                                   arrowprops=dict(arrowstyle='-', color=color, lw=0.5))
            fig.canvas.draw()
            best_bbox = best_ann.get_window_extent(renderer=renderer)
        placed_bboxes.append(best_bbox)

# Subplot 2: Velocity
ax2 = plt.subplot(3, 1, 2)
vel_series = []
for ds in datasets:
    times = [m[0] / 1000.0 for m in ds["data"]]
    velocities = [m[12] for m in ds["data"]]
    line, = ax2.plot(times, velocities, label=ds["label"])
    vel_series.append((times, velocities, line.get_color()))
ax2.set_ylabel('Velocity (m/s)')
configure_time_axis(ax2)
ax2.legend()
annotate_maxes(ax2, vel_series)

# Subplot 3: Height
ax3 = plt.subplot(3, 1, 3)
height_series = []
for ds in datasets:
    times = [m[0] / 1000.0 for m in ds["data"]]
    heights = [m[9] for m in ds["data"]]
    line, = ax3.plot(times, heights, label=ds["label"])
    height_series.append((times, heights, line.get_color()))
ax3.set_ylabel('Height (m)')
ax3.set_xlabel('Time (s)')
configure_time_axis(ax3)
ax3.legend()
annotate_maxes(ax3, height_series)

plt.tight_layout()
plt.show()

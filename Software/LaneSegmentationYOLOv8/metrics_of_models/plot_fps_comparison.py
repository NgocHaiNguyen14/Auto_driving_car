import matplotlib.pyplot as plt

# Data for the models
models = ['yolo-large', 'yolo-medium', 'yolo-small','Mask-RCNN']  # Names of the models
fps = [28, 37, 46,21]  # FPS for each model
avg_inference_times = [0.0346, 0.0261, 0.0208, 0.0461]  # Average inference times in seconds

# Create a figure and axis
fig, ax1 = plt.subplots()

# Plot FPS as a bar chart
color = 'tab:blue'
bars = ax1.bar(models, fps, color=color, alpha=0.6, label='FPS')
ax1.set_xlabel('Model')
ax1.set_ylabel('FPS', color=color)
ax1.tick_params(axis='y', labelcolor=color)

# Annotate bars with integer FPS values
for bar in bars:
    yval = bar.get_height()
    ax1.text(bar.get_x() + bar.get_width() / 2, yval, f'{int(round(yval))}', va='bottom', ha='center', color=color)

# Create a second y-axis to plot average inference time
ax2 = ax1.twinx()
color = 'tab:red'
ax2.set_ylabel('Average Inference Time (s)', color=color)
ax2.plot(models, avg_inference_times, color=color, marker='o', label='Average Inference Time (s)')
ax2.tick_params(axis='y', labelcolor=color)
ax2.set_ylim(0, max(avg_inference_times) + 0.01)

# Add titles and labels
plt.title('Comparison of YOLOv8 Models Performance')
fig.tight_layout()  # To ensure the labels fit into the plot
plt.grid()

# Save the plot as a PNG file
plt.savefig('fps_comparison.png')

# Show the plot
plt.show()
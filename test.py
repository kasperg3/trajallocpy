import matplotlib.pyplot as plt

# List of points
points = [
    (39.12644037002258, 93.71578621411169),
    (41.5, 100.0),
    (41.5, 0.0),
    (20.2908, 6.79677),
    (12.7317, 18.113),
    (12.7317, 18.113),
    (9.28861, 7.24275),
    (9.28861, 1.87095),
    (9.28861, 7.24275),
    (11.5, 14.224375),
    (11.5, 100.0),
    (56.8659, 71.2941),
    (53.5, 100.0),
    (53.5, 0.0),
]
# Separate x and y coordinates
x_values = [point[0] for point in points]
y_values = [point[1] for point in points]

# Plot the points
plt.scatter(x_values, y_values)

# Plot lines between the points
for i in range(len(points) - 1):
    plt.plot([points[i][0], points[i + 1][0]], [points[i][1], points[i + 1][1]], "b-")

# Add labels and title
plt.xlabel("X")
plt.ylabel("Y")
plt.title("Scatter Plot with Lines between Points")

# Display the plot
plt.show()

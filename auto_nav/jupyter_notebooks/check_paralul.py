import numpy as np
from sklearn.linear_model import RANSACRegressor
from joblib import Parallel, delayed
import matplotlib.pyplot as plt

# Generate a synthetic dataset of multiple parallel lines
np.random.seed(42)
n_lines = 5
n_points = 1000
x = np.random.uniform(low=-10, high=10, size=n_points)
y = np.random.uniform(low=-10, high=10, size=n_points)
print(x)
print(y)

X = np.vstack([x, y]).T
coefs = np.random.uniform(low=-1, high=1, size=n_lines)
y_true = np.dot(X, coefs.reshape(n_lines, 2).T).flatten()
y_obs = y_true + np.random.normal(loc=0, scale=3, size=n_points)


# Define the RANSAC fitting function for a single line
def fit_ransac(data):
    X, y = data
    ransac = RANSACRegressor()
    ransac.fit(X, y)
    return ransac.estimator_.coef_

# Define the parallel RANSAC fitting function for multiple lines
def parallel_ransac(X, y, n_lines, n_jobs=2, batch_size=1000):
    # Split the data into batches
    X_batches = [X[i:i+batch_size] for i in range(0, len(X), batch_size)]
    y_batches = [y[i:i+batch_size] for i in range(0, len(y), batch_size)]

    # Run RANSAC on each batch in parallel
    results = Parallel(n_jobs=n_jobs)(delayed(fit_ransac)((X_batch, y_batch)) for X_batch, y_batch in zip(X_batches, y_batches))

    # Merge the results and return the fitted coefficients
    return np.vstack(results).reshape(n_lines, 2)

# Fit the lines using parallel RANSAC
coefs_ransac = parallel_ransac(X, y_obs, n_lines=n_lines)

# Plot the results
plt.scatter(X[:, 0], y_obs, s=5, color='blue')
for i in range(n_lines):
    plt.plot(X[:, 0], np.dot(X, coefs_ransac[i]), linewidth=2, color='red')
plt.show()

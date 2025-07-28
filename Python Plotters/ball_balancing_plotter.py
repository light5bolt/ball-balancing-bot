import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

data = pd.read_csv("data.csv")
print(data)

D = data.to_numpy()
t = D[:,0]
ball_x = D[:,1]
ball_y = D[:,2]
target_x = D[:,3]
target_y = D[:,4]

plt.plot(ball_x, ball_y)
plt.plot(target_x, target_y)
plt.show()
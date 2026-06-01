
import numpy as np
import matplotlib.pyplot as plt
import math
import imageio.v2 as imageio
import os

N = 400
c = 2
A = 0.000005
G = 0.01

p_array = np.zeros([N+1,N+1],float)
print(p_array[0][50])
L = 99
d = L/(N-1)
dt = 0.02

p_env = 1
p_env_array = np.ones([N+1,N+1],float)*p_env

dp_dt = np.zeros([N+1,N+1],float)
d2p_dt2 = np.zeros([N+1,N+1],float)
dp_dx = np.zeros([N+1,N+1],float)
dp_dy = np.zeros([N+1,N+1],float)
d2p_dx2 = np.zeros([N+1,N+1],float)
d2p_dy2 = np.zeros([N+1,N+1],float)

def precompute_dp_dx(p_array, dp_dx):
    dx = 1*d
    dp_dx[1:-1, :] = p_array[1:-1, :] - p_array[0:-2, :]
    dp_dx *= 1/dx

def precompute_dp_dy(p_array, dp_dy):
    dy = 1*d
    dp_dy[:, 1:-1] = p_array[:, 1:-1] - p_array[:, 0:-2]
    dp_dy *= 1/dy

def precompute_d2p_dx2(p_array, d2p_dx2):
    dx = 1*d
    d2p_dx2[1:-1, :] = dp_dx[2:, :] - dp_dx[1:-1, :]
    d2p_dx2 *= 1/dx

def precompute_d2p_dy2(p_array, d2p_dy2):
    dy = 1*d
    d2p_dy2[:, 1:-1] = dp_dy[:, 2:] - dp_dy[:, 1:-1]
    d2p_dy2 *= 1/dy

# initial conditions:

# for x in range(40, 71):
#   p_array[x] = [0.1*math.cos((x-55)*math.pi/30) for y in range (N+1)]

# for x in range(N+1):
#     p_array[x] = [p_env for y in range(N+1)]
# for x in range(10, 50):
#     # p_array[x] = [0.1*math.cos((x-60)*math.pi/40) for y in range(N+1)]
#     for y in range(N+1):
#       p_array[x,y] += A*math.cos((x-30)*math.pi/20)
    # dp_dt[x] = [A*10*math.pi/20*math.sin((x-30)*math.pi/20)]

# for x in range(0, N):
#   p_array[x] = [A*math.sin(x*math.pi/N) for y in range (N+1)]
#   dp_dt[x] = [0 for y in range (N+1)]

# for x in range(0, N):
#   p_array[x] += [-1*A*math.sin(x*math.pi/(N/2)) for y in range (N+1)]
#   dp_dt[x] += [-1*A*c*math.pi/(N/2)*math.cos(x*math.pi/(N/2))]

for x in range(N+1):
    p_array[x] = [p_env for y in range(N+1)]
for x in range(181, 221):
  for y in range(0, 21):
    r = ((x-200)**2 + (y-10)**2)**0.5
    if r <= 10:
      p_array[x][y] += -1*A*math.cos(r*math.pi/20)

# boundary = [[N/2-(N**2/4 - ((i-N/2)**2))**0.5, N/2+(N**2/4 - ((i-N/2)**2))**0.5] for i in range(N)]
# outside_boundary = [[float(y in boundary or y < boundary[x][0] or y > boundary[x][1]) for y in range(N)] for x in range(N)]
# print(boundary[80])
# print(outside_boundary)

# plt.imshow(p_array, cmap="rainbow")

results = []
def time_step(p_array, dp_dt, dt):
    d2p_dt2 = c**2*(d2p_dx2 + d2p_dy2) + 4*math.pi*G*p_env*(p_array-p_env_array)
    dp_dt += d2p_dt2*dt
    p_array += dp_dt*dt
    update_boundaries(p_array)

def update_boundaries(p_array):
    # # soft boundary condition:
    # p_array[0,:] = p_array[1,:]
    # p_array[-1,:] = p_array[-2,:]

    # hard boundary condition:
    dp_dt[0,:] = dp_dt[1,:]
    dp_dt[-1,:] = dp_dt[-2,:]

t = 0
T = 40
num_frames = 200
total_steps = T/dt
print(p_array-p_env_array)
counter = total_steps/num_frames
frame_count = 0

while t < T:
    if counter < 0:
        counter = total_steps/num_frames
        print("frame: " + str(frame_count))
        frame_count += 1
    precompute_dp_dx(p_array, dp_dx)
    precompute_d2p_dx2(p_array, d2p_dx2)
    precompute_dp_dy(p_array, dp_dy)
    precompute_d2p_dy2(p_array, d2p_dy2)
    time_step(p_array, dp_dt, dt)
    if (round(t/dt))%(total_steps//num_frames)==0:
        results.append(p_array.copy())
    t += dt
    counter -= 1

print(p_array[55][10])
print(d2p_dx2[55][10])
print(dp_dt[55][10])

print(p_array[55][10])
print(len(results))
print(0.04//0.02)
print(total_steps//num_frames)

# for i in range(100):
#   fig, ax = plt.subplots()

#   im = ax.imshow(results[i], cmap='rainbow')

#   filename = "/content/output/frame_" + str(i) + ".png"
#   plt.savefig(filename, dpi=150, bbox_inches='tight')

#   plt.close(fig)  # VERY important to free memory
scale = 5
for i in range(100):
    scaling = np.kron(results[i], np.ones((scale, scale)))
    filename = "output/frame_" + str(i) + ".png"
    # plt.imsave(filename, scaling, cmap='rainbow', vmin=0.9, vmax=1.1)
    plt.imsave(filename, scaling, cmap='RdBu_r', vmin=0.999997, vmax=1.000003)


input_dir = "output"
output_path = "results/jeans4.gif"

# Collect and sort filenames properly
filenames = sorted(
    [f for f in os.listdir(input_dir) if f.endswith(".png")],
    key=lambda x: int(x.split("_")[1].split(".")[0])
)

# Convert filenames to full paths
filepaths = [os.path.join(input_dir, f) for f in filenames]

# Create GIF at 30 fps
with imageio.get_writer(output_path, mode='I', duration=1/30) as writer:
    for filepath in filepaths:
        image = imageio.imread(filepath)
        writer.append_data(image)

print("GIF saved to:", output_path)

x = np.linspace(0, N, N+1)
y = np.linspace(0, N, N+1)
X, Y = np.meshgrid(x,y)

fig, ax = plt.subplots()
CS = ax.contour(X,Y,p_array,2)
ax.clabel(CS,fontsize=10)
ax.imshow(p_array,cmap='rainbow', vmin=-0.1, vmax=0.1)
# cbar = fig.colorbar(im, ax=ax)

plt.savefig("results/contour_frame.png", dpi=300, bbox_inches='tight')
plt.close()

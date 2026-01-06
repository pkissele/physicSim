from PIL import Image
from sys import argv

numframes = 321
prefix = 'frame_'
ani = []
durationLen = 500
folderName = 'output'

if len(argv) > 1:
    folderName = argv[1]
if len( argv ) > 2:
    numframes = int(argv[2])
if len(argv) > 3:
    durationLen = float(argv[3])

for j in range(numframes):
    num = f'{j:05d}'
    img = Image.open(f'{folderName}/{prefix}{num}.png')
    ani.append(img.copy())

ani[0].save('movie.gif', save_all = True, append_images = ani, duration = durationLen, loop = 1)

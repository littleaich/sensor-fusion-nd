import os
import imageio

img_dir = '../build';
gif_filename = 'ukf.gif';

assert os.path.isdir(img_dir);

images = [];

file_list = [];

with imageio.get_writer(gif_filename, mode='I', fps=50) as writer :
	for i in range(1,300) :
		img = imageio.imread(os.path.join(img_dir, 'frame_' + str(i) + '.png'));
		writer.append_data(img);

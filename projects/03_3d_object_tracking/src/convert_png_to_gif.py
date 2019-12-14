import os
import imageio

img_dir = '../build';
gif_filename = 'ttc.gif';

assert os.path.isdir(img_dir);

images = [];

file_list = [];

with imageio.get_writer(gif_filename, mode='I', fps=3) as writer :
	for i in range(1,19) :
		img = imageio.imread(os.path.join(img_dir, 'image-' + str(i) + '.png'));
		writer.append_data(img);

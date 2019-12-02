import os
import imageio

img_dir = '../build';
gif_filename = 'detection.gif';

assert os.path.isdir(img_dir);

images = [];

file_list = [];
for fname in os.listdir(img_dir) :
	if fname.startswith('frame_') :
		file_list.append(fname);

with imageio.get_writer(gif_filename, mode='I', fps=5) as writer :
	for fname in sorted(file_list) :
		img = imageio.imread(os.path.join(img_dir, fname));
		writer.append_data(img);

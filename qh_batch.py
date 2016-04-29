import os
# for nn in range(500, 10000, 500):
for ss in range(1, 11):
	os.system("cargo run --release -- -n 10000 -s {0}".format(ss))
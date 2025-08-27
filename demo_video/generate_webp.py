from PIL import Image
import imageio.v2 as imageio
import glob, os, re

solutions = list(range(1, 9))
cols, rows = 4, 2
fps = 30
outname = "ur5e_grid_pingpong.webp"
quality = 80

files = sorted(glob.glob("ur5e_1_*.png"))
frame_numbers = [int(re.search(r"_(\d+)\.png$", f).group(1)) for f in files]

CROP = 50  # pixels per side
MASK_W, MASK_H = 125, 110  # bottom-left mask

frames = []
for idx in frame_numbers:
    tiles = []
    for sol in solutions:
        fname = f"ur5e_{sol}_{idx:03d}.png"
        if not os.path.exists(fname):
            raise FileNotFoundError(fname)
        im = Image.open(fname).convert("RGB")

        # 1) bottom-left white rectangle (on the full-size tile)
        w, h = im.size
        im.paste((255, 255, 255), (0, h - MASK_H, MASK_W, h))

        # 2) crop 50 px from all sides
        left   = min(CROP, w//2 - 1)
        right  = max(w - CROP, left + 1)
        top    = min(CROP, h//2 - 1)
        bottom = max(h - CROP, top + 1)
        im = im.crop((left, top, right, bottom))

        tiles.append(im)

    tw, th = tiles[0].size
    grid = Image.new("RGB", (cols * tw, rows * th), "white")
    for j, im in enumerate(tiles):
        x = (j % cols) * tw
        y = (j // cols) * th
        grid.paste(im, (x, y))

    frames.append(grid)

seq = frames + frames[-2:0:-1]

imageio.mimsave(
    outname,
    seq,
    fps=fps,
    loop=0,
    format="WEBP",
    quality=quality
)

print(f"Saved {outname}, {len(seq)} frames.")

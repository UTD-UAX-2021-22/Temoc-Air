import cv2
import PIL

ids = [list(range(x*4,4*(x+1))) for x in range(7)]
ppi_val = 20

def gen_board(ids, ppi):
    
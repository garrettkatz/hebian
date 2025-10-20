import numpy as np
import matplotlib.pyplot as pt
from qreader import QReader
import cv2

# Create a QReader instance
qreader = QReader()

# Get the image that contains the QR code
# fname = "qr_wikip_op.png"
# fname = "qr_close.jpg"
fname = "qr_med.jpg"
# fname = "qr_far.jpg"
image = cv2.cvtColor(cv2.imread(fname), cv2.COLOR_BGR2RGB)

pt.imshow(image)

# detect the qr codes
hits = qreader.detect(image)

for hit in hits:
    quad = hit["quad_xy"]
    print(quad)
    outline = np.concatenate([quad, quad[:1]], axis=0)
    pt.plot(*outline.T, 'b.-')

pt.show()

# # Use the detect_and_decode function to get the decoded QR data
# decoded_text = qreader.detect_and_decode(image=image)

# for text in decoded_text:
#     print(text)
# print(len(decoded_text))

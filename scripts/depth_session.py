# coding: utf-8
imgmsg.encoding
import numpy as np
temp =  np.fromstring(imgmsg.data)
temp.shape
imgmsg.height
imgmsg.height*imgmsg.width
temp =  np.fromstring(imgmsg.data, dtype=np.uint16)
temp.shape
temp[0]
temp.mean()
temp.max()
temp.median()
np.median(temp)
temp_re = temp.reshape(imgmsg.height, imgmsg.width)
import matplotlib.pyplot as plt
plt.imshow(temp_re)
plt.show()
np.percentile(temp, 90)
np.percentile(temp, 99)
get_ipython().magic(u'save depth_session.py')
get_ipython().magic(u'save depth_session.py 1-21')

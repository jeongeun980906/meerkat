import PIL.Image as pilimg
import numpy as np
import keras
import tensorflow as tf
import math

num=3
test_array=np.zeros((num,28,28,1))
dir0='/home/jhmbabo/ml/num_image/num_image_54.jpg'
test_img0=pilimg.open(dir0)
test_array0= np.array(test_img0)
dir1='/home/jhmbabo/ml/num_image/num_image_53.jpg'
test_img1=pilimg.open(dir1)
test_array1= np.array(test_img1)
dir2='/home/jhmbabo/ml/num_image/num_image_55.jpg'
test_img2=pilimg.open(dir2)
test_array2= np.array(test_img2)

test_array0=test_array0.reshape(28,28,1)
test_array[0]=test_array0
test_array1=test_array1.reshape(28,28,1)
test_array[1]=test_array1
test_array2=test_array2.reshape(28,28,1)
test_array[2]=test_array2
test_array=test_array/255.0

model=tf.keras.models.load_model('sibal.hdf5')
prediction=model.predict(test_array)
result=0
for i,prediction in enumerate(prediction):
    temp=np.argmax(prediction)
    res=temp.item()
    print(type(res))
    result+=math.pow(10,i)*res
print(result)
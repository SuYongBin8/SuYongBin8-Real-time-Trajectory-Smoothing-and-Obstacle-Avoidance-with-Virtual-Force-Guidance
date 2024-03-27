# from PIL import Image
#
# # 读取两张图片
# img1 = Image.open('plot1.png')
# img2 = Image.open('plotm.png')
#
# # 计算图片的高度比例
# height_ratio = img2.height / img1.height
#
# # 调整图片的大小
# img1_resized = img1.copy()
# img2_resized = img2.copy()
# img1_resized = img1_resized.crop((0, 0, int(img1_resized.width * height_ratio), img1_resized.height))
# img2_resized = img2_resized.crop((0, 0, int(img2_resized.width * height_ratio), img2_resized.height))
#
# # 获取图片的宽度
# width1, width2 = img1_resized.width, img2_resized.width
#
# # 创建新的画布
# concatenated_img = Image.new('RGB', (width1 + width2, img1_resized.height), (255, 255, 255))
#
# # 拼接图片
# concatenated_img.paste(img1_resized, (0, 0))
# concatenated_img.paste(img2_resized, (width1, 0))
#
# # 保存拼接后的图片
# concatenated_img.save('56.png')


#纵向拼接

from PIL import Image

# 读取两张图片
img1 = Image.open('plot1-15.png')
img2 = Image.open('plot2-15.png')

# 计算图片的宽度比例
width_ratio = img2.width / img1.width

# 调整图片的大小
img1_resized = img1.copy()
img2_resized = img2.copy()
img1_resized = img1_resized.crop((0, 0, img1_resized.width, int(img1_resized.height * width_ratio)))
img2_resized = img2_resized.crop((0, 0, img2_resized.width, int(img2_resized.height * width_ratio)))

# 获取图片的高度
height1, height2 = img1_resized.height, img2_resized.height
print(img1_resized.width,height1, height2)
# 创建新的画布
concatenated_img = Image.new('RGB', (img1_resized.width, height1 + height2), (255, 255, 255))

# 拼接图片
concatenated_img.paste(img1_resized, (0, 0))
concatenated_img.paste(img2_resized, (0, height1))
print(concatenated_img.width,concatenated_img.height)
# 保存拼接后的图片
concatenated_img.save('plot12.png')


#
# #小图贴到大图上
# from PIL import Image
#
# # 打开大图和小图
# large_image = Image.open("123.png")
# small_image = Image.open("plot12.png")
# # 计算小图的新宽度，使得长度为1000像素，宽度按比例缩放
# width_percent = 1000 / small_image.size[0]
# new_width = int(small_image.size[0] * width_percent)
# new_height = int(small_image.size[1] * width_percent)
#
# # 缩放小图
# small_image = small_image.resize((new_width, new_height))
# # 将小图粘贴到大图的指定位置
# position = (2300, 1800)  # 小图在大图中的左上角位置
# large_image.paste(small_image, position)
#
# # 保存合成后的图像
# large_image.save("combined_image.png")
#
# # 显示合成后的图像
# large_image.show()

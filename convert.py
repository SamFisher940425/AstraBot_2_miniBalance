import os
import codecs

# 遍历目录下所有的文件
def batch_encoding_conversion(path):
    # 遍历目录下的文件
    for root, dirs, files in os.walk(path):
        print (f"dir:{dirs} files:{files}")
        for file in files:
            file_path = os.path.join(root, file)
            if file.endswith('.c') or file.endswith('.h'):   # 只处理c/h文件，可以根据需要修改
                # 对于gb2312编码格式的文件进行转换 and 'gb2312' in open(file_path).read()
                with codecs.open(file_path, 'r', 'gb2312') as f:
                    content = f.read()
                    # 把编码从gb2312转为utf-8
                    content_gb = content.encode('utf-8', 'ignore').decode('utf-8')
                    with codecs.open(file_path, 'w', 'utf-8') as f2:
                        f2.write(content_gb)

batch_encoding_conversion('.')
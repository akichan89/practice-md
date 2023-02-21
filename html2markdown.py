import markdownify # 👉️ pip install markdownify
import os
import os.path


def convert_html():
  # with open('lmpt_payload_controls.html','r+') as firstfile, open('lmpt_payload_controls.md','w') as secondfile:
  with open('messages/_html/common.html','r+') as firstfile, open('common.md','w') as secondfile:
  
    data = ' '
    for line in firstfile:
        data = data+line
      
    convert = markdownify.markdownify(data, heading_style="ATX") # 👉️ Convert
    secondfile.write(convert)
    secondfile.close()
    firstfile.close()            
print("done")

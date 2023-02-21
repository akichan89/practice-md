import markdownify # 👉️ pip install markdownify
import os
import os.path


def convert_html(path, filename):
  # install = 'sudo apt install pandoc wkhtmltopdf'
  # os.system(install)
  # with open('lmpt_payload_controls.html','r+') as firstfile, open('lmpt_payload_controls.md','w') as secondfile:
  with open(path,'r+') as firstfile, open(filename,'w') as secondfile:
  
    data = ' '
    for line in firstfile:
        data = data+line
      
    convert = markdownify.markdownify(data, heading_style="ATX") # 👉️ Convert
    secondfile.write(convert)
    
    secondfile.close()
    firstfile.close()  
    newname = filename.replace('.md', '.pdf')
    print(newname)
    cmd = f'pandoc {filename}  --from=gfm --pdf-engine=wkhtmltopdf --metadata pagetitle="Primary Payload" --output {newname}'  
    os.system(cmd)

    # if (os.path.exists('/home/autumn/saved-pdk/')):
    #   cd = 'cd /home/autumn/saved-pdk'
    #   remove = 'sudo rm common.md all.md lmpt_generic_component_controls.md common.pdf all.pdf generic.pdf pdk-archive.tar.gz'
    #   os.system(cd)
    #   print(cd)
    #   os.system(remove)
    #   print(remove)
    # else:
    #   mkdir = 'sudo mkdir /home/autumn/saved-pdk cd /home/autumn/saved-pdk'
    #   os.system(mkdir)
    # compress = 'tar -czvf pdk-archive.tar.gz /home/autumn/saved-pdk/'
    # os.system(compress)
    # print(compress)
    # copy = 'sudo cp common.md all.md lmpt_generic_component_controls.md common.pdf all.pdf generic.pdf /home/autumn/saved-pdk/'
    # os.system(copy)
    # print(copy)
    if (os.path.exists('/home/autumn/dev-pdk/practice-md/saved_files')):
      os.system('cp all.md all.pdf common.md common.pdf test.md test.pdf saved_files')
    else:
      os.mkdir('saved_files')
      os.system('cp all.md all.pdf common.md common.pdf test.md test.pdf saved_files')
print("done")

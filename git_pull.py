import os # for walk

def git_pull():
    os.chdir('/home/autumn/dev-pdk/message_definitions/v1.0')
    cmd = 'git pull origin'
    os.system(cmd)
    print(cmd)
    os.chdir('/home/autumn/dev-pdk/practice-md')
    main = 'pandoc intro.md  --from=gfm --pdf-engine=wkhtmltopdf --metadata pagetitle="Primary Payload" --output main.pdf' 
    os.system(main)
    print(main)
    print("pulled")

git_pull()

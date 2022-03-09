import argparse
import os
import re
import time
import tempfile
import shutil
from datetime import datetime
import shutil
import git

# Imports required for emailing
import smtplib, ssl, socket
import html as htmlLib
from email.mime.text import MIMEText
from email.mime.multipart import MIMEMultipart

#from watchdog.observers import Observer
#from watchdog.events import PatternMatchingEventHandler

# TODO: something to detect if your stuck and display clippy telling you to take a break

def parse_args():
    def dir_path(string):
        if os.path.isdir(string):
            return string
        else:
            raise NotADirectoryError(string)

    parser = argparse.ArgumentParser(description='Watch changes to code & create git commit for every save')
    parser.add_argument('watch', type=dir_path, nargs="?", default=os.getcwd(), help='your project directory to watch (default=cwd)')
    parser.add_argument('--email', nargs="*", default=['cplepel@imsa.edu', 'dsingh@imsa.edu', 'scoutinho@imsa.edu'], help="list of people to email the report to once it's done, separated by spaces")
    parser.add_argument('--frequency', '-f', nargs="?", type=int, default=60, help='How many seconds between commits (default=60')

    # Default args: r"C:\Users\Ethan\Documents\GitHub\FRC-JAVA-2022 --email cplepel@imsa.edu dsingh@imsa.edu scoutinho@imsa.edu".split(" ")
    return parser.parse_args()


# Adapted from https://thepythoncorner.com/posts/2019-01-13-how-to-create-a-watchdog-in-python-to-look-for-filesystem-changes/
def watch():
    patterns = ["*"]
    ignore_patterns = None
    ignore_directories = False
    case_sensitive = True
    '''def on_created(event):
        print(f"hey, {event.src_path} has been created!")

    def on_deleted(event):
        print(f"what the f**k! Someone deleted {event.src_path}!")

    def on_modified(event):
        print(dir(event))
        print(f"hey buddy, {event.src_path} has been modified")

    def on_moved(event):
        print(f"ok ok ok, someone moved {event.src_path} to {event.dest_path}")'''

    ''' Not realy nessisary to watch files as long as I copy them over every minute
    my_event_handler = PatternMatchingEventHandler(patterns, ignore_patterns, ignore_directories, case_sensitive)
    my_event_handler.on_created = on_created
    my_event_handler.on_deleted = on_deleted
    my_event_handler.on_modified = on_modified
    my_event_handler.on_moved = on_moved
    path = args.watch
    go_recursively = True
    my_observer = Observer()
    my_observer.schedule(my_event_handler, path, recursive=go_recursively)
    print("Watching", args.watch + "...")
    my_observer.start()
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("Stopping...")
        my_observer.stop()
        my_observer.join()'''


# This doesn't work too well, still returns duplicate commits
"""def getDiff(compare1=0, compare2=1):
    diff = os.popen(f"git diff HEAD~{compare1} HEAD~{compare2} --unified=0").read()
    print('\n'.join(re.findall("^[\+-](?!\+\+)(?!--).*", diff, re.MULTILINE)))
    # in bash, I would pipe into grep -Po '(?<=^\+)(?!\+\+).*'
    print(diff)
    return diff"""


def diffAll():
    allCommits = []
    commits_list = list(repo.iter_commits())

    # Compare the current HEAD against the bare init commit
    for commitIndex in range(len(commits_list) - 1):
        a_commit = commits_list[commitIndex + 1]
        b_commit = commits_list[commitIndex]

        changes = a_commit.diff(b_commit, create_patch=True)[0].diff.decode("utf-8")
        changes = '\n'.join(re.findall("^[\+-](?!\+\+)(?!--).*", changes, re.MULTILINE))
        
        allCommits.append(a_commit.summary + "\n" + changes)
    
    return allCommits


def applyColor(html):
    """Makes added lines green, and deleted lines red"""
    html = htmlLib.escape(html)
    # g<0> is equivalent to $& in JS or $0 in PCRE
    html = re.sub("^\+.*", '<span style="color: green">\\g<0></span>', html, flags=re.MULTILINE)
    html = re.sub("^-.*", '<span style="color: red">\\g<0></span>', html, flags=re.MULTILINE)
    return "<pre>" + html + "</pre>"
    

def email(html):
    # DO NOT COMMIT TO GIT, CONTAINS SECRETS
    port = 465  # For SSL
    addr = "edawes@imsa.edu"
    password = "idwdmiagperccxda"
    # Thanks to https://stackoverflow.com/a/12422921
    recipients = args.email

    # Create a secure SSL context
    context = ssl.create_default_context()

    message = MIMEMultipart("alternative")
    message["Subject"] = "Robotics Hours Request"
    message["From"] = addr
    message["To"] = ", ".join(recipients)
    message.attach(MIMEText(html, "html"))
    # print(message.as_string())

    with smtplib.SMTP_SSL("smtp.gmail.com", port, context=context) as server:
        server.login(addr, password)
        server.sendmail(
            addr, recipients, message.as_string()
        )
    print("Email sent!")


def checkChanges():
    # I'm not sure if using hard links will reduce the strain on my SSD
    # Unfortunately, `copy_function=os.link` errors if file already exists
    shutil.copytree(args.watch, tempdir.name, ignore=shutil.ignore_patterns(".git"), dirs_exist_ok=True)  # Docs: https://docs.python.org/3/library/shutil.html#shutil.copytree
    timestamp = datetime.now().strftime("%c")
    # TODO: file deletion/addition is untracked
    try:
        repo.git.add(".")  # equivalent to os.system("git add .")
        repo.git.commit(m="Checkpoint " + timestamp)  # equivalent to os.system('git commit -m "Checkpoint ' + timestamp + '"')
    except git.exc.GitCommandError:
        print("No changes detected")  # Do nothing

    print(datetime.now().strftime("%X") + " | Checking for changes...")


def main():
    pass


if __name__ == "__main__":
    args = parse_args()
    startTime = time.time()
    main()

    tempdir = tempfile.TemporaryDirectory()
    os.chdir(tempdir.name)
    print(os.popen("echo %cd%").read())
    repo = git.Repo.init()
    #os.system("git init")

    while True:
        checkChanges()
        try:
            for _ in range(args.frequency):  # Broken into 1 second intervals so that I can KayboardInterrupt early
                time.sleep(1)
        except KeyboardInterrupt:
            checkChanges()
            break
        # Don't do this because code may not have changed, resulting in duplicated commits
        """if (diff := getDiff()):
            allChanges += diff + '\n'"""

    try:
        print("Stopping...")
        allChanges = applyColor('\n\n'.join(diffAll()))
        accomplish = input("What did you accomplish?: ")
        msg = f'<p>Hello, I am requesting {(time.time() - startTime) / 3600} hours that I spent programming for robotics while not in the lab. I worked on "{accomplish}". Following is a list of the changes I made every {args.frequency} seconds:</p>' + allChanges + "<p>Thank you 🤖</p>"
        retry = 0
        if len(args.email) != 0:
            for retry in range(1, 4):  # Will retry 3 times, increasing wait time by 20 sec each time
                try:
                    email(msg)
                    break
                except socket.gaierror:
                    waitfor = retry * 10
                    print("Network error, retrying in", waitfor, "seconds")
                    time.sleep(waitfor)
        if len(args.email) == 0 or retry == 3:
            print("\nUnable to send, here is the HTML body:")
            print(allChanges)
        # Zipping & sending the git history seems like overkill, but still an option
        # shutil.make_archive("git", 'zip', ".git")
    finally:  # No error is expected to be thrown, moreso for debugging errors
        repo.__del__()
        os.chdir("/")
        tempdir.cleanup()

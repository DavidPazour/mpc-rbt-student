#!/bin/bash

function scriptRootDir() {
  #https://stackoverflow.com/questions/59895/how-can-i-get-the-source-directory-of-a-bash-script-from-within-the-script-itsel/246128
  echo "$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
}

receiverProjectPath="$(scriptRootDir)"/example_receiver.service
senderProjectPath="$(scriptRootDir)"/example_sender.service
receiverServicePath=/etc/systemd/system/example_receiver.service
senderServicePath=/etc/systemd/system/example_sender.service

echo -e "\n\tLinking service configs:"
sudo ln --symbolic --force --verbose  "$receiverProjectPath" "$receiverServicePath"
sudo ln --symbolic --force --verbose  "$senderProjectPath" "$senderServicePath"

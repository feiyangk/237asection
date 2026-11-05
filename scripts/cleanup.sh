#!/usr/bin/sh
git config --global --unset user.name
git config --global --unset user.email
gh auth logout

rm -r ~/autonomy_ws


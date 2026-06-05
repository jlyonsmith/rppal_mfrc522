#!/usr/bin/env fish

set bin_name $argv[1]

if test -z $bin_name
    echo "Usage: $argv[0] BIN_NAME"
    exit 1
end

# Change the ownership of the files
sudo chown root:root $bin_name

# Move files into place
sudo mv $bin_name /usr/local/bin/$bin_name

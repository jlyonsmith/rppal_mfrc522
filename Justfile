list:
	just --list

test:
	#!/usr/bin/env fish
	set LOOPS 5
	set MFRC522 'target/aarch64-unknown-linux-gnu/debug/mfrc522'
	cargo build
	echo Reader J1 ------------------------------
	$MFRC522 -0 pin17 -0 pin27 -0 pin22 -r pin20 --loops $LOOPS
	echo Reader J2 ------------------------------
	$MFRC522 -1 pin17 -0 pin27 -0 pin22 -r pin20 --loops $LOOPS
	echo Reader J3 ------------------------------
	$MFRC522 -0 pin17 -1 pin27 -0 pin22 -r pin20 --loops $LOOPS
	echo Reader J4 ------------------------------
	$MFRC522 -1 pin17 -1 pin27 -0 pin22 -r pin20 --loops $LOOPS
	echo Reader J5 ------------------------------
	$MFRC522 -0 pin17 -0 pin27 -1 pin22 -r pin20 --loops $LOOPS
	echo Reader J6 ------------------------------
	$MFRC522 -1 pin17 -0 pin27 -1 pin22 -r pin20 --loops $LOOPS
	echo Reader J7 ------------------------------
	$MFRC522 -1 pin17 -1 pin27 -0 pin22 -r pin20 --loops $LOOPS
	echo Reader J8 ------------------------------
	$MFRC522 -1 pin17 -1 pin27 -1 pin22 -r pin20 --loops $LOOPS

coverage OPEN='':
  #!/usr/bin/env fish
  set -x RUSTFLAGS '-Cinstrument-coverage'
  set -x LLVM_PROFILE_FILE (pwd)'/scratch/'(whoami)'-%p-%m.profraw'
  # Using the for loop avoids warnings in output about non-existent files
  for file in (pwd)/scratch/*.profraw; rm $file; end
  cross test --tests --target aarch64-unknown-linux-gnu
  grcov . --source-dir . --binary-path ./target/aarch64-unknown-linux-gnu/debug \
    --output-types html --branch --ignore-not-existing --output-path ./target/debug/coverage \
    --excl-start '^//\s*\{grcov-excl-start\}' --excl-stop '^//\s*\{grcov-excl-end\}'
  cp ./target/debug/coverage/coverage.json ./coverage.json
  if string match -r 'open$' -- '{{OPEN}}'
    open target/debug/coverage/index.html
  end

doc OPEN='':
  #!/usr/bin/env fish
  if string match -r 'open$' -- '{{OPEN}}'
    cargo doc --open
  else
    cargo doc
  end

release OPERATION='incrPatch':
  #!/usr/bin/env fish
  function info
    set_color green; echo "👉 "$argv; set_color normal
  end
  function warning
    set_color yellow; echo "🐓 "$argv; set_color normal
  end
  function error
    set_color red; echo "💥 "$argv; set_color normal
  end

  if test ! -e "Cargo.toml"
    error "Cargo.toml file not found"
    exit 1
  end

  info "Checking for uncommitted changes"

  if not git diff-index --quiet HEAD -- > /dev/null 2> /dev/null
    error "There are uncomitted changes - commit or stash them and try again"
    exit 1
  end

  set branch (string trim (git rev-parse --abbrev-ref HEAD 2> /dev/null))
  set name (basename (pwd))

  info "Starting release of '"$name"' on branch '"$branch"'"

  info "Checking out '"$branch"'"
  git checkout $branch

  info "Pulling latest"
  git pull

  mkdir scratch 2> /dev/null

  if not stampver {{OPERATION}} -u -i version.json5
    error "Unable to generation version information"
    exit 1
  end

  set tagName (cat "scratch/version.tag.txt")
  set tagDescription (cat "scratch/version.desc.txt")

  git rev-parse $tagName > /dev/null 2> /dev/null
  if test $status -ne 0; set isNewTag 1; end

  if set -q isNewTag
    info "'"$tagName"' is a new tag"
  else
    warning "Tag '"$tagName"' already exists and will not be moved"
  end

  if test -e 'justfile' -o -e 'Justfile'
    just coverage
  else
    cross test --target aarch64-unknown-linux-gnu
  end

  if test $status -ne 0
    # Rollback
    git checkout $branch .
    error "Tests failed '"$name"' on branch '"$branch"'"
    exit 1
  end

  info "Staging version changes"
  git add :/

  info "Committing version changes"
  git commit -m $tagDescription

  if set -q isNewTag
    info "Tagging"
    git tag -a $tagName -m $tagDescription
  end

  info "Pushing to 'origin'"
  git push --follow-tags

  info "Finished release of '"$name"' on branch '"$branch"'. You can publish the crate."
  exit 0

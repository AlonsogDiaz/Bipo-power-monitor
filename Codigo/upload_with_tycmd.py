Import("env")

env.Replace(
    UPLOADER="\"c:\\Program Files (x86)\\TyTools\\tycmd.exe\"",
    UPLOADCMD="$UPLOADER upload $UPLOADERFLAGS $SOURCE"
)
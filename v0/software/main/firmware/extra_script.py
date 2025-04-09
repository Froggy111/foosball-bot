import os
Import("env")

# include toolchain paths
env.Replace(COMPILATIONDB_INCLUDE_TOOLCHAIN=True)

# override compilation DB path
en.Replace(COMPILATIONDB_PATH="compile_commands.json")

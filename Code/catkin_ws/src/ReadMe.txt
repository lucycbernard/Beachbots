Remember: 
    When creating a new ros node in a package
    1. Add the line "#!/usr/bin/env python" (without the quotes) to the top of the file
    2. in the terminal, in the directory of the file $ chmod +x file.python
    3. in the package's CMakeLists.txt, add the python file in the appropriate area

    When creating a new ros launch file
    1. in the package's CMakeLists.txt, add the launch file in the appropriate area
<center> 

# Common commands and setup scripts
***
</center>

## Installing uv via winget
- **run the following command in the terminal:**
    `winget install --exact --disable-interactivity --accept-source-agreements --accept-package-agreements --silent --id=astral-sh.uv`
## Setting up your cloned repository [^1]
[^1]: This will be using your newly installed uv

<small> All of the following commands should be run while in your FRC-2026 folder/directory 
(generally in C:\repos\FRC-2026) </small>

- **run once per cloned repository/FRC-2026 folder:** 
    <small>installs tool that checks for errors in code before commiting:</small>
        `uv tool run pre-commit`
    &#8201;
    <small>creates a virtual environment to prevent problems from global packages:</small>
        `uv venv`
    &#8201;
    <small>sets the virtual environment as the defaul python profile:</small>
        `.venv\Scripts\activate`
## common commands after setup

**sync packages:**
    `uv run robotpy --main src sync`
    <small>downloads and installs packages included in robotpy</small>

**run simulation:** 
    `uv run robotpy --main src sim`
    <small>simulates how the code would run if connected to the robot</small>

**deploy code:** 
    `uv run robotpy --main src deploy`
    <small>deploys the code to the robot</small>

**undeploy code:** 
    `uv run robotpy --main src undeploy`
    <small>clears the code last deployed to the robot</small>

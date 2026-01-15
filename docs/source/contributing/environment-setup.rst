Environment Setup
=================

We use `Sphinx <https://www.sphinx-doc.org/>`__ for
building and formatting the html files. To add to
the documentation, we use a combination of VSCode extensions
and terminal commands which allow a quick and simple way
of writing documentation and viewing it live.

Prerequisites
-------------

Before starting this setup guide, ensure that the following programs and extensions are installed.
For the VSCode extensions, install them in the same instance as where the rest of the control code
is edited.

- `Python 3.12 <https://www.python.org/downloads/>`__ (ensure that it is in path)
- `Make <https://gnuwin32.sourceforge.net/packages/make.htm>`__ (only for windows)
- `Python extension <https://marketplace.visualstudio.com/items?itemName=ms-python.python>`__
- `Esbonio <https://marketplace.visualstudio.com/items?itemName=swyddfa.esbonio>`__
- `reStructuredText <https://marketplace.visualstudio.com/items?itemName=lextudio.restructuredtext>`__
- `reStructuredText Syntax <https://marketplace.visualstudio.com/items?itemName=trond-snekvik.simple-rst>`__
- `Live Preview <https://marketplace.visualstudio.com/items?itemName=ms-vscode.live-server>`__
- `File Watcher <https://marketplace.visualstudio.com/items?itemName=appulate.filewatcher>`__

.. note::
    Don't forget to add the ``make`` binary to path, it is called during building.

Python
------

Since Sphinx is based on python, it is required to have the correct packages installed.
First, restart VSCode to restart all of the extensions. Then, open the ``Command Palette``
and run ``Python: Select Interpreter``. Choose the python version you installed as the
prerequisite and hit enter.

.. important::

  There is a known bug with the ``Esbonio`` extension where it does not support spaces
  in the python binary path. Make sure to select an installation in ``Program Files``
  on any other directory without a space

After selecting the python installation, open terminal by pressing :kbd:`Ctrl+Shift+`` and
run the following commands.

.. code-block:: pwsh

  cd docs
  pip install -r requirements_dev.txt

Finally, restart VSCode to re-run the python extensions.

Configuration
-------------
.. note::

  This step is optional since there is already a ``settings.json`` in the project,
  however if something does not work, make sure to check that all of these settings are correct.

Once all of the extensions are installed, open the **user** ``settings.json`` from
the ``Command Palette`` :kbd:`Ctrl+Shift+P` and make sure to set the esbonio and lint paths.

.. code-block:: json

    "restructuredtext.linter.doc8.extraArgs": [
      "--config",
      "${workspaceFolder}/docs/doc8.ini"
    ],
    "esbonio.sphinx.numJobs": 0,
    "esbonio.sphinx.buildDir": "${workspaceFolder}\\docs\\build",
    "esbonio.sphinx.confDir": "${workspaceFolder}\\docs\\source"

To view the ``rst`` file updates live, we use the ``filewatcher`` and ``livePreview`` extensions
together.

.. code-block:: json

    "filewatcher.commands": [
      {
        "event": "onFileChange",
        "match": "\\.rst*",
        "cmd":"\"${workspaceRoot}\\docs\\make.bat\" html"
      }
    ],
    "livePreview.defaultPreviewPath": "docs/build/html/index.html",
    "livePreview.previewDebounceDelay": 3000

Additionally, to have the preview automatically change as you type, set the autosave
delay around ``1000``

.. code-block:: json

    "files.autoSave": "afterDelay",
    "files.autoSaveDelay": 1000

Editing
-------

When editing the documentation, open the ``Command Palette`` by pressing :kbd:`Ctrl+Shift+P`
and run ``Live Preview: Start Server`` while an ``rst`` file is open.

.. note::

 If the live preview window can't find the html file, try to cause a rebuild by editing
 a ``rst`` file. Make sure that the ``docs/build`` directory is being created since that
 is where the live preview points to. Finally, if it still doesn't work, open the output panel
 by pressing :kbd:`Ctrl+Shift+U` and select ``Esbonio`` from the dropdown
 in the top right. If everything is working, the output should show something about ``build succeeded.``.

.. _manual-building:

Manual Building
---------------

If you just pulled new changes or want to preview the documentation without live editing the files,
it is possible to build it through the terminal.

.. code-block:: pwsh

  cd docs
  ./make html # run make.bat, if using command prompt ./ is not needed

The output html files should be put in the same ``docs/build`` directory
that the live preview builds in. Now you can open them in a browser or
start the Live Preview server to view them inside VSCode.

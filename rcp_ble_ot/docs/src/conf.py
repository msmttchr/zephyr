# Configuration file for the Sphinx documentation builder.
#
# For the full list of built-in configuration values, see the documentation:
# https://www.sphinx-doc.org/en/master/usage/configuration.html

# -- Project information -----------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#project-information

project = 'RCP BLE and Openthread'
copyright = '2026, STMicroelectronics'
author = 'STMicroelectronics'
release = '0.1'

# -- General configuration ---------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#general-configuration

extensions = []

templates_path = ['_templates']
exclude_patterns = ['_build', 'Thumbs.db', '.DS_Store']



# -- Options for HTML output -------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#options-for-html-output

html_theme = 'bizstyle'
#html_static_path = ['_static']
html_theme_options = {
    # Disable showing the sidebar. Defaults to 'false'
    'nosidebar': True,
}
html_permalinks = False
html_use_index = False
# This will add "Last updated on: Jan 14, 2026" (or current date) to the footer
# html_last_updated_fmt = '%b %d, %Y'
#
# Optional: If you want to show the specific time as well:
html_last_updated_fmt = '%Y-%m-%d %H:%M:%S'

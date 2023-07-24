#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# Configuration file for the Sphinx documentation builder.
#
# For the full list of built-in configuration values, see the documentation:
# https://www.sphinx-doc.org/en/master/usage/configuration.html


import os
import sys

sys.path.insert(0, os.path.abspath('../../../'))
sys.path.insert(0, os.path.abspath('../../../commonroad_crime/'))

# -- Project information -----------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#project-information


project = 'CommonRoad-CriMe'
copyright = '2023, Technical University of Munich, Cyber-Physical Systems Group'
author = 'Yuanfei Lin, Matthias Althoff'
release = 'v0.2.4'

# -- General configuration ---------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#general-configuration

# Add any Sphinx extension module names here, as strings. They can be
# extensions coming with Sphinx (named 'sphinx.ext.*') or your custom
# ones.
extensions = ["sphinx.ext.autodoc",
              "sphinx.ext.viewcode",
              "sphinx.ext.napoleon",
              'sphinx.ext.inheritance_diagram',
              "sphinx.ext.todo",
              'sphinx_autodoc_typehints',
              "m2r2"
              ]

# order of displaying the members in an automodule/autoclass
autodoc_member_order = "bysource"

# show todo items
todo_include_todos = True

autodoc_typehints = "both"

# -- Options for HTML output -------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#options-for-html-output

html_theme = 'sphinx_rtd_theme'

html_theme_options = {
    'canonical_url': '',
    'analytics_id': '',
    'logo_only': False,
    'display_version': True,
    'prev_next_buttons_location': 'bottom',
    'style_external_links': False,
    # Toc options
    'collapse_navigation': False,
    'sticky_navigation': True,
    'navigation_depth': 4,
    'includehidden': True,
    'titles_only': False
}

html_logo = 'img/CriMe-logo.png'

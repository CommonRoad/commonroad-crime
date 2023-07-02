#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# Configuration file for the Sphinx documentation builder.
#
# For the full list of built-in configuration values, see the documentation:
# https://www.sphinx-doc.org/en/master/usage/configuration.html

# -- Project information -----------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#project-information

project = 'CommonRoad-CriMe'
copyright = '2023, Yuanfei Lin, Liguo Chen, Kun Qian, Ziqian Xu, Sicheng Wang'
author = 'Liguo Chen, Kun Qian, Ziqian Xu, Sicheng Wang'
release = 'v0.2.4'

# -- General configuration ---------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#general-configuration

extensions = [
		'sphinx.ext.autodoc',
		'sphinx.ext.viewcode'
		]
exclude_patterns = []
todo_include_todos = False


# -- Options for HTML output -------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#options-for-html-output

html_theme = "sphinx_rtd_theme" #pip install sphinx_rtd_theme

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

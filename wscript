#!/usr/bin/env python
# encoding: utf-8
# waf configuration file .waf --help pour commande par défaut
# Utilisé par CMD dist

## Par défaut ./waf configure va utiliser buildir=wbuild et CXX=g++
## MAIS avec ./waf configure --out=cbuild --check-cxx-compiler=clang++
##      on peut utilise clang :o)
## l'option --color permet de coloriser la sortie du compilo

APPNAME = 'GDyn'
VERSION = '1.0'

# root, build dir
top = '.'
out = 'wbuild'

opt_flags = '-O3'
debug_flags = '-O0 -g'

# ************************************************************************* help
def help(ctx):
    print( "**** WAF for VisuGL, usual commands ************" )
    print( "configuration :  ./waf configure --out=cbuild --check-cxx-compiler=clang++" )
    print( "build :          ./waf build ")
    print( "build specific : ./waf build --targets=test/001-curve" )
    print( "clean :          ./waf clean" )
    print( "detailed help :  ./waf --help or see https://waf.io/book" )
    print( "  options :      --compil_db --atb --debug" )

# ********************************************************************** options
def options( opt ):
    opt.load( 'compiler_cxx' )

    # option debug
    opt.add_option('--debug', dest='debug', action="store_true", default=False,
                   help='compile with debugging symbols' )

    # clang compilation database
    opt.add_option('--compil_db', dest='compil_db', action="store_true", default=False,
                   help='use clang compilation database' )

    # define some macro for C++
    # (equivalent to #define LABEL or -DLABEL to compilator
    opt.add_option('-D', '--define', action="append", dest="defined_macro",
                   help='define preprocessing macro' )

# **************************************************************** CMD configure
def configure( conf ):
    conf.load( 'compiler_cxx' )

    if conf.options.compil_db:
        ## To generate 'compile_commands.json' at the root of buildpath
        # to be linked/copied in order to user 'cquery' in emacs through lsp-mode
        # see https://github.com/cquery-project/cquery/wiki/Emacs
        conf.load('clang_compilation_database', tooldir="ressources")
        print( "CXX=",conf.env.CXX)
    
    conf.env['CXXFLAGS'] = ['-D_REENTRANT','-Wall','-fPIC','-std=c++20']
    #.conf.env.INCLUDES_PROJETS = conf.path.abspath()+"/../include"

# ******************************************************************** CMD build
def build( bld ):
    print('→ build from ' + bld.path.abspath())

    # check debug option
    if bld.options.debug:
        bld.env['CXXFLAGS'] += debug_flags.split(' ')
    else:
        bld.env['CXXFLAGS'] += opt_flags.split(' ')
    print( bld.env['CXXFLAGS'] )
        
    
    bld.recurse( 'examples' )
    # bld.recurse( 'doc' )

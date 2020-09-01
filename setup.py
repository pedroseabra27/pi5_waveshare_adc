from setuptools import setup, Extension, find_packages
from io import open
from os import path
from Cython.Distutils import build_ext
from os.path import dirname, join

# __version__ is imported by exec, but help linter not complain
__version__ = None
exec(open("pi_waveshare_adc/_version.py", encoding="utf-8").read())

here = path.abspath(dirname(__file__))

with open(join(here, 'README.rst'), encoding='utf-8') as f:
    long_description = f.read()

cmdclass = {'build_ext': build_ext}
libraries = ['wiringPi', 'pigpio']
ext_modules = [
    Extension(
        'pi_waveshare_adc._adc',
        sources=[join(here, 'pi_waveshare_adc', '_adc.pyx')],
        libraries=libraries),
]

for e in ext_modules:
    e.cython_directives = {"embedsignature": True, 'language_level': 3}

URL = 'https://github.com/matham/pi_waveshare_adc'

setup(
    name='pi_waveshare_adc',
    version=__version__,
    author='Matthew Einhorn',
    author_email='moiein2000@gmail.com',
    license='LGPL-2.1',
    description='A performant Python library to read the RPi Waveshare '
                'ADC expansion board.',
    long_description=long_description,
    url=URL,
    classifiers=[
        'Development Status :: 4 - Beta',
        'License :: OSI Approved :: MIT License',
        'Programming Language :: Python :: 3.6',
        'Programming Language :: Python :: 3.7',
        'Programming Language :: Python :: 3.8',
    ],
    packages=find_packages(),
    cmdclass=cmdclass,
    ext_modules=ext_modules,
    project_urls={
        'Bug Reports': URL + '/issues',
        'Source': URL,
    },
)

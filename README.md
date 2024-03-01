# Advisor Project 2023/24
This year we are supposed to build a maze solving robot similar to [Micromouse](https://de.wikipedia.org/wiki/Micromouse). Except our robot has to be capable of picking up and carrying a ball.

## Why C++
C/C++ are the intended languages to program a stm32 MCU. We use C++ as it's more powerful, while still being extremly fast and providing the memory management capabilites of C.
### Ressources
- [The Cherno](https://www.youtube.com/@TheCherno) is a great youtube channel if you want to learn about certain C++ features.
- If you know Java you can search for guides like "learn c++ from java". Yet there are quite a few concepts in C++ that don't exist in Java.
- Here is a list of a few additional concepts you should understand: <b>Pointers and References</b>, Preprocessor directives, Macros, <b>Header files</b>, Namespaces, etc. 
### IDE/Code Editor
I recommend using Visual Studio Code (Code Editor). It's lightweight yet has powerful features. If you plan to use an IDE, please adjust the .gitignore file accordingly. Otherwise the project will have so many unnecessary files. Note however that the "motors & sensors group" will have a different file structure as I assume they will be using STM32CubeIDE. However, the proposed workflow should be the same and they should just have a seperate repository or we'll create a combined repository where we seperate the two file structures. 

## Unit testing
[Unit testing](https://en.wikipedia.org/wiki/Unit_testing) is a common method to ensure that every component of a larger application is working as intended. It's done by writing test cases for every "unit" (which is just a section of code). For example if you write a function called "factorial(n)" you would add a test case which checks whether that functions returns the correct value for a few test inputs and all edge cases. This might seem unncessary and not worth the time, but when working on a larger project with multiple members it is an absolute must have. We will be constantly changing code that was written by others and to ensure that the functionality that they implemented is still working properly we need independent test cases which will verify the validity of the changes.
### How?
In order to create proper unit tests, we need a unit testing framework. Common choices for C++ are [GoolgeTest](https://github.com/google/googletest), [Catch2](https://github.com/catchorg/Catch2), [CppUnit](https://cppunit.sourceforge.net/doc/1.8.0/) and [doctest](https://github.com/doctest/doctest). My suggestion would be to use doctest. It has several benefits compared to others:
1. It's very easy to setup (one header file only)
2. It has extremly fast compilation & execution times (saves free minutes for github actions)
3. It's very easy to create a new test case
4. It's thread safe (although we hopefully won't need this)
5. Doesn't produce warnings (even on the most agressive warning levels) 

Here is a simple example for a test case:
```c++
#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"

int factorial(int number) { 
    if(number <= 1) {
        return 1;
    }
    return number * factorial(number - 1); 
}

TEST_CASE("testing the factorial function") {
    CHECK(factorial(0) == 1);
    CHECK(factorial(1) == 1);
    CHECK(factorial(2) == 2);
    CHECK(factorial(3) == 6);
    CHECK(factorial(10) == 3628800);
}
```
As you can see, defining a test case is just a matter of including the header file and some Macros. I highly recommend having a look at [the docs](https://github.com/doctest/doctest/tree/master/doc/markdown). It's not necessary to read everything but looking at the [tutorial](https://github.com/doctest/doctest/blob/master/doc/markdown/tutorial.md) and the [types of assertions](https://github.com/doctest/doctest/blob/master/doc/markdown/assertions.md) available, are a good starting point.

## Collabroative Working
The following is a suggestion on what a possible workflow could look like. It's important to give feedback, so that we can adjust our workflow according to our needs. It's important to understand concepts related to git and GitHub like commits, push/pull, branches, merges and pull requests for the following.
### Worfklow
Let's say you want to add a new feature. In this case you want to create a function, sumUpTo(n), which sums up all the numbers from 1 to n.
1. Create a new branch
```shell
git checkout -b create_sumUpTo
```
2. Create the function
```c++
int sumUpTo(int n) {
    return (n <= 1) ? 1 : (n + sumUpTo(n-1));
}
```
3. Create a test
```c++
TEST_CASE("testing the sumUpTo function") {
    CHECK(sumUpTo(1) == 1);
    CHECK(sumUpTo(2) == 3);
    CHECK(sumUpTo(3) == 6);
    CHECK(sumUpTo(-10) == 1);
}
```
4. Run all tests (could look similar to this)
```shell
make test #compiles tests (as specified in Makefile)
build/main #optional: run application (located in folder build)
build/test #runs test file in folder build
```
4. Commit and Push
```shell
git add .
git commit -m "added sumUpTo function"
git push origin create_sumUpTo
```
5. Create a pull request on GitHub
6. GitHub actions will automatically run all tests again and check for merge conflicts (which you might have to resolve)
7. Wait for your pull request to be reviewed by another team member.
8. After the review the pull request will be merged into the main branch and the feature branch will be deleted.
9. Optional: Delete the feature branch locally
```shell
git branch -d create_sumUpTo
``` 
10. Pull latest changes of the main branch locally (including your recently merged feature branch)
```shell
git checkout main
git pull origin main
```
11. Repeat

### Github Actions
With GitHub Actions we can automatically create a test envoirment that runs our code and most importantly runs the tests we defined. Creating a pull request is an event that triggers a workflow, which sets up a linux machine, then compiles and runs our tests. Furthermore we can verify that the pull request won't cause any merge conflict. This is necessary information to approve a pull request that is merged into the main branch because <b>we want to ensure that the main branch is always working correctly</b>. Don't worry about setting that up, you don't have to. 

### Organize taks
We need some tool that helps us keep track which tasks are due and who is working on what. We could use somthing like [Trello](https://trello.com/de) or [Notion](https://www.notion.so)

### Consistency
Consistency is very important to keep things simple. Language is one aspect to consider. I would suggest that we use englisch because everyone understands it and otherwise we would probably end up with a mixture of german and english which just creates unnecessary complexity. Furthermore it might be wise to use a [Linter](https://www.testautomatisierung.org/lexikon/linting/) and/or a code formatter to achieve a more readable and consistent code style.  

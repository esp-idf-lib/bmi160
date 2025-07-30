# About `template-component`

This is a template repository for `esp-idf-lib` component. The repository is
intended to be cloned to create new component quickly.

<!-- vim-markdown-toc GFM -->

* [Features](#features)
* [Using the template](#using-the-template)
    * [Creating a repository from the template](#creating-a-repository-from-the-template)
    * [Enable GitHub Pages](#enable-github-pages)
    * [Protect the default branch](#protect-the-default-branch)
* [Obtain an access token of ESP Component Registry](#obtain-an-access-token-of-esp-component-registry)
* [Initial commit](#initial-commit)

<!-- vim-markdown-toc -->

## Features

* CI is enabled by default
* The documentation is published to GitHub Pages
* The component is published to ESP Component Repository when released

## Using the template

Before committing and pushing to the GitHub repository, the repository must be
configured.

### Creating a repository from the template

Follow the official documentation, [Creating a repository from a template](https://docs.github.com/en/repositories/creating-and-managing-repositories/creating-a-repository-from-a-template#creating-a-repository-from-a-template).

### Enable GitHub Pages

Visit [Settings], [Pages] in the side menu at left, and choose [GitHub Actions]
as [Source] under [Build and deployment].

With `gh` CLI command:

```sh
gh api repos/:owner/:repo/pages -X POST -F build_type=workflow
```

### Protect the default branch

1. Visit [Settings].
1. Click [Rules] in the side menu at left and click [Rulesets].
1. Click [New ruleset] button, and click [New branch ruleset].
1. [New branch ruleset] page is shown.
1. In [Ruleset Name], type "default"
1. Under [Enforcement status], select [Active]
1. Under [Target branches], click [Add target] button, and select [Include
   default branch].
1. Under [Branch rules], make sure the default rules, [Restrict deletions] and
   [Block force pushes], are checked.
1. Check [Require a pull request before merging]. Leave its options as-is.

## Obtain an access token of ESP Component Registry

To automate uploading a component to ESP Component Registry with GitHub
Actions workflow, you need:

* to be an admin of the component on ESP Component Registry
* to have a privilege to set Actions secrets and variables in the GitHub
  repository.

Contact @trombik if you don't or are not sure.

To use a workflow to upload the component when the component has been released,
you need an access token of ESP Component Registry. The token is used in a
workflow via GitHub secrets.


1. Visit [ESP Component Registry](https://components.espressif.com/) and
   login.
1. Click your profile menu on top and click [Tokens]
1. In [Create new access token] page, fill in [Description], e.g. "Uploading
   component from GitHub Action".
1. Select [write:component] in [Scopes of the token].
1. Click [Create]
1. In [Access token successfully created] page, copy the generated access
   token. Please keep it secret and do not publish it anywhere. This token is
   used in the next step and shown just once.

Next, create a GitHub secret in the repository.

1. Visit the component repository on GitHub.
1. Click [Settings]
1. Click [Secrets and variables] in the side menu at left.
1. Under [Repository secrets], click [New repository secret]
1. In [Actions secrets / New secret] page, fill in [Name] with `ESP_TOKEN`
1. Paste the access token value from the previous step into [Secret].
1. Click [Add secret].
1. Make sure the token name, `ESP_TOKEN`, is shown in [Repository secrets]

## Initial commit

After all of the above, you can start committing and pushing your code to the
repository.

For documenting the component, see `docs` directory.

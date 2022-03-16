# Rules for commits

## The 7 rules for commit messages 

1. Separate subject from body with a blank line
2. Limit the subject line to 50 characters 
3. Capitalize the subject line
4. Do not end the subject line with a period
5. Use the imperative mood in the subject line
6. Wrap the body at 72 characters (if change is too simple to add a further context, leave out body)
7. Use the body to explain what and why vs. how


## Things to avoid when creating commits

- Mixing whitespace changes with functional code changes
- Mixing two unrelated functional changes
- Sending large new features in a single giant commit
	
Basic rule to follow:  
If a code change can be split into a sequence of patches/commits, then it should be split. Less is not more. More is more.


## Information in commit messages

- Describe why a change is being made.
- How does it address the issue?
- What effects does the patch have?
- Do not assume the reviewer understands what the original problem was.
- Do not assume the code is self-evident/self-documenting.
- Read the commit message to see if it hints at improved code structure.
- The first commit line is the most important.
- Describe any limitations of the current code.
- Do not include patch set-specific comments.


## Example

Summarize changes in around 50 characters or less

More detailed explanatory text, if necessary. Wrap it to about 72  
characters or so. In some contexts, the first line is treated as the  
subject of the commit and the rest of the text as the body. The  
blank line separating the summary from the body is critical (unless  
you omit the body entirely); various tools like `log`, `shortlog`  
and `rebase` can get confused if you run the two together.  

Explain the problem that this commit is solving. Focus on why you  
are making this change as opposed to how (the code explains that).  
Are there side effects or other unintuitive consequences of this  
change? Here's the place to explain them.

Further paragraphs come after blank lines.

- Bullet points are okay, too
- Typically a hyphen or asterisk is used for the bullet, preceded  
  by a single space, with blank lines in between, but conventions  
  vary here  

If you use an issue tracker, put references to them at the bottom,  
like this:

Resolves: #123  
See also: #456, #789

## Sources


[1]	Atlassian; https://at.projects.genivi.org/wiki/display/PROJ/Rules+for+Commit+Messages , [last viewed 17.05.2019]  
[2] Beams, Chris; https://chris.beams.io/posts/git-commit/ , [last viewed 17.05.2019]  
[3] Openstack; https://wiki.openstack.org/wiki/GitCommitMessages#Information_in_commit_messages , [last viewed 17.05.2019]  

namespace :demo do
  task :deploy do
    sh "rm -Rf _site"
    sh "jekyll --url http://iaroc.org"
    sh "rsync -va --delete _site/ ssh.example.com:staging/htdocs/"
  end
end

namespace :prod do
  task :deploy do
    sh "rm -Rf _site"
    sh "jekyll --url http://iaroc.org"
    sh "rsync -va --delete _site/ ssh.example.com:htdocs/"
  end
end

function out = compute_glcm(image)
  
  m = graycomatrix(image);

  out = m(:)' / sum(m(:));

end
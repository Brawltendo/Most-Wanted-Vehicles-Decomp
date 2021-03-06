/*
 * Copyright (c) 1999
 * Silicon Graphics Computer Systems, Inc.
 *
 * Copyright (c) 1999 
 * Boris Fomitchev
 *
 * This material is provided "as is", with absolutely no warranty expressed
 * or implied. Any use is at your own risk.
 *
 * Permission to use or copy this software for any purpose is hereby granted 
 * without fee, provided the above notices are retained on all copies.
 * Permission to modify the code and to distribute modified code is granted,
 * provided the above notices are retained, and a notice that the code was
 * modified is included with the above copyright notice.
 *
 */ 
// WARNING: This is an internal header file, included by other C++
// standard library headers.  You should not attempt to use this header
// file directly.


#ifndef _STLP_INTERNAL_ISTREAMBUF_ITERATOR_H
#define _STLP_INTERNAL_ISTREAMBUF_ITERATOR_H

#ifndef _STLP_INTERNAL_ITERATOR_BASE_H
# include <stl/_iterator_base.h>
#endif

#ifndef _STLP_INTERNAL_STREAMBUF
# include <stl/_streambuf.h>
#endif

_STLP_BEGIN_NAMESPACE

// defined in _istream.h
template <class _CharT, class _Traits>
extern basic_streambuf<_CharT, _Traits>* _STLP_CALL _M_get_istreambuf(basic_istream<_CharT, _Traits>& ) ;

// We do not read any characters until operator* is called. operator* calls sgetc 
// unless the iterator is unchanged from the last call in which case a cached value is
// used. Calls to operator++ use sbumpc.

template<class _CharT, class _Traits>
class istreambuf_iterator
{
public:
  typedef _CharT                           char_type;
  typedef _Traits                          traits_type;
  typedef typename _Traits::int_type       int_type;
  typedef basic_streambuf<_CharT, _Traits> streambuf_type;
  typedef basic_istream<_CharT, _Traits>   istream_type;

  typedef input_iterator_tag               iterator_category;
  typedef _CharT                           value_type;
  typedef typename _Traits::off_type       difference_type;
  typedef const _CharT*                    pointer;
  typedef const _CharT&                    reference;

public:
  istreambuf_iterator(streambuf_type* __p = 0) { this->_M_init(__p); }
  //  istreambuf_iterator(basic_istream<_CharT, _Traits>& __is) { this->_M_init(_M_get_istreambuf(__is)); }
  inline istreambuf_iterator(basic_istream<_CharT, _Traits>& __is);

  char_type operator*() const { this->_M_getc(); return _M_c; }
  istreambuf_iterator<_CharT, _Traits>& operator++() { this->_M_bumpc(); return *this; }
  istreambuf_iterator<_CharT, _Traits>  operator++(int);

  bool equal(const istreambuf_iterator<_CharT, _Traits>& __i) const {
    if (this->_M_buf)
      this->_M_getc();
    if (__i._M_buf)
      __i._M_getc(); 
    return this->_M_eof == __i._M_eof;
  }

private:
  void _M_init(streambuf_type* __p) {
    _M_buf = __p;
    _M_eof = !__p;
    //    _M_is_initialized = _M_eof;
    _M_have_c = false;
  }

  void _M_getc() const {
    if (_M_have_c)
      return;
    int_type __c = _M_buf->sgetc();
# if !defined (_STLP_NEED_MUTABLE) /* && ! defined (__SUNPRO_CC) */
    _M_c = traits_type::to_char_type(__c);
    _M_eof = traits_type::eq_int_type(__c, traits_type::eof());
    _M_have_c = true;
# else
    typedef istreambuf_iterator<_CharT,_Traits> _Self;
    _Self* __that = __CONST_CAST(_Self*, this);
    __that->_M_c = __STATIC_CAST(_CharT, traits_type::to_char_type(__c));
    __that->_M_eof = traits_type::eq_int_type(__c, traits_type::eof());
    __that->_M_have_c = true;
# endif
  }

  void _M_bumpc() {
    _M_buf->sbumpc();
    _M_have_c = false;
  }

private:
  streambuf_type* _M_buf;
  mutable _CharT _M_c;
  mutable unsigned char _M_eof;
  mutable unsigned char _M_have_c;
};

template<class _CharT, class _Traits>
inline istreambuf_iterator<_CharT, _Traits>::istreambuf_iterator(basic_istream<_CharT, _Traits>& __is) 
{ this->_M_init(_M_get_istreambuf(__is)); }

template<class _CharT, class _Traits>
inline bool _STLP_CALL operator==(const istreambuf_iterator<_CharT, _Traits>& __x,
                                  const istreambuf_iterator<_CharT, _Traits>& __y) {
  return __x.equal(__y);
}

#ifdef _STLP_USE_SEPARATE_RELOPS_NAMESPACE

template<class _CharT, class _Traits>
inline bool _STLP_CALL operator!=(const istreambuf_iterator<_CharT, _Traits>& __x,
                                  const istreambuf_iterator<_CharT, _Traits>& __y) {
  return !__x.equal(__y);
}

#endif /* _STLP_USE_SEPARATE_RELOPS_NAMESPACE */

# if defined (_STLP_USE_TEMPLATE_EXPORT)
_STLP_EXPORT_TEMPLATE_CLASS istreambuf_iterator<char, char_traits<char> >;
#  if defined (INSTANTIATE_WIDE_STREAMS)
_STLP_EXPORT_TEMPLATE_CLASS istreambuf_iterator<wchar_t, char_traits<wchar_t> >;
#  endif
# endif /* _STLP_USE_TEMPLATE_EXPORT */

# ifdef _STLP_USE_OLD_HP_ITERATOR_QUERIES
template <class _CharT, class _Traits>
inline input_iterator_tag _STLP_CALL iterator_category(const istreambuf_iterator<_CharT, _Traits>&) { return input_iterator_tag(); }
template <class _CharT, class _Traits>
inline streamoff _STLP_CALL 
distance_type(const istreambuf_iterator<_CharT, _Traits>&) { typedef streamoff __off_type; return __off_type(); }
template <class _CharT, class _Traits>
inline _CharT* _STLP_CALL value_type(const istreambuf_iterator<_CharT, _Traits>&) { return (_CharT*)0; }
# endif

template <class _CharT, class _Traits>
istreambuf_iterator<_CharT, _Traits>
istreambuf_iterator<_CharT, _Traits>::operator++(int) {
  istreambuf_iterator<_CharT, _Traits> __tmp = *this;
  this->_M_bumpc();
  this->_M_have_c = false;
  return __tmp;
}

_STLP_END_NAMESPACE

#endif /* _STLP_INTERNAL_ISTREAMBUF_ITERATOR_H */

// Local Variables:
// mode:C++
// End:


/*
 * L.Control.GeoSearch - search for an address and zoom to its location
 * https://github.com/smeijer/leaflet.control.geosearch
 */

L.GeoSearch = {};
L.GeoSearch.Provider = {};

L.GeoSearch.Result = function (x, y, label) {
  this.X = x;
  this.Y = y;
  this.Label = label;
};

L.Control.GeoSearch = L.Control.extend({
  options: {
    position: 'topleft'
  },

  initialize: function (options) {
    this._config = {};
    L.Util.extend(this.options, options);
    this.setConfig(options);
  },

  setConfig: function (options) {
    this._config = {
      'provider': options.provider,
      'searchLabel': options.searchLabel || 'Enter address',
      'notFoundMessage' : options.notFoundMessage || 'Sorry, that address could not be found.',
      'zoomLevel': options.zoomLevel || 17,
      'showMarker': typeof options.showMarker !== 'undefined' ? options.showMarker : true
    };
  },

  resetLink: function(extraClass) {
    var link = this._container.querySelector('a');
    link.className = 'leaflet-bar-part leaflet-bar-part-single' + ' ' + extraClass;
  },

  onAdd: function (map) {

    // create the container
    this._container = L.DomUtil.create('div', 'leaflet-bar leaflet-control leaflet-control-geosearch');

    // create the link - this will contain one of the icons
    var link = L.DomUtil.create('a', '', this._container);
    link.href = '#';
    link.title = this._config.searchLabel;

    // set the link's icon to magnifying glass
    this.resetLink('glass');

    var displayNoneClass = 'displayNone';

    // create the form that will contain the input
    var form = L.DomUtil.create('form', displayNoneClass, this._container);

    // create the input, and set its placeholder ("Enter address") text
    var input = L.DomUtil.create('input', null, form);
    input.placeholder = 'Enter address';

    // create the error message div
    var message = L.DomUtil.create('div', 'leaflet-bar message displayNone', this._container);

    L.DomEvent
    .on(link, 'click', L.DomEvent.stopPropagation)
    .on(link, 'click', L.DomEvent.preventDefault)
    .on(link, 'click', function() {

      if (L.DomUtil.hasClass(form, displayNoneClass)) {
	L.DomUtil.removeClass(form, 'displayNone'); // unhide form
	input.focus();
      } else {
	L.DomUtil.addClass(form, 'displayNone'); // hide form
      }

    })
    .on(link, 'dblclick', L.DomEvent.stopPropagation);

    L.DomEvent
    .on(input, 'keypress', this.onKeyPress, this)
    .on(input, 'keyup', this.onKeyUp, this)
    .on(input, 'input', this.onInput, this);

    return this._container;
  },

  geosearch: function (qry) {
    try {
      var provider = this._config.provider;

      if(typeof provider.GetLocations == 'function') {
	var results = provider.GetLocations(qry, this._map, function(err, results) {
	  if (err) {
	    return this._printError(err);
	  }

	  this._processResults(results);
	}.bind(this));
      }
      else {
	var url = provider.GetServiceUrl(qry);

	$.getJSON(url, function (data) {
	  try {
	    var results = provider.ParseJSON(data);
	    this._processResults(results);
	  }
	  catch (error) {
	    this._printError(error);
	  }
	}.bind(this));
      }
    }
    catch (error) {
      this._printError(error);
    }
  },

  _processResults: function(results) {
    if (results.length === 0)
      throw this._config.notFoundMessage;

    this.cancelSearch();
    this._showLocation(results[0]);
  },

  _showLocation: function (location) {
    if (this._config.showMarker) {
      if (typeof this._positionMarker === 'undefined')
	this._positionMarker = L.marker([location.Y, location.X]).addTo(this._map);
      else
	this._positionMarker.setLatLng([location.Y, location.X]);
    }

    // this._map.setView([location.Y, location.X], this._config.zoomLevel, false);
  },

  _isShowingError: false,

  _printError: function(error) {
    var message = this._container.querySelector('.message');
    message.innerHTML = error;
    L.DomUtil.removeClass(message, 'displayNone');

    // show alert icon
    this.resetLink('alert');

    this._isShowingError = true;
  },

  cancelSearch: function() {
    var form = this._container.querySelector('form');
    L.DomUtil.addClass(form, 'displayNone'); // hide form

    var input = form.querySelector('input');
    input.value = ''; // clear form

    // show glass icon
    this.resetLink('glass');

    var message = this._container.querySelector('.message');
    L.DomUtil.addClass(message, 'displayNone'); // hide message
  },

  startSearch: function() {
    // show spinner icon
    this.resetLink('spinner');

    var input = this._container.querySelector('input');
    this.geosearch(input.value);
  },

  onInput: function() {
    if (this._isShowingError) {
      // show glass icon
      this.resetLink('glass');

      var message = this._container.querySelector('.message');
      L.DomUtil.addClass(message, 'displayNone'); // hide message

      this._isShowingError = false;
    }
  },

  onKeyPress: function (e) {
    var enterKey = 13;

    if (e.keyCode === enterKey) {
      L.DomEvent.preventDefault(e); // prevent default form submission

      this.startSearch();
    }
  },

  onKeyUp: function (e) {
    var escapeKey = 27;

    if (e.keyCode === escapeKey) {
      this.cancelSearch();
    }
  }
});
